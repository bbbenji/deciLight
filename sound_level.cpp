#include "sound_level.h"

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

#include "config.h"

// NOTE: sos-iir-filter.h emits the filter kernel as file-scope assembly, so it
// defines symbols rather than declaring them. Include it from this translation
// unit only, or the link will fail with duplicate definitions.
#include "sos-iir-filter.h"

namespace sound_level {
namespace {

// -----------------------------------------------------------------------------
// Filters
//
// Second-Order Sections cascades, assuming b0 and a0 are 1.0 with the gain
// applied in the last step. Coefficients come from the Octave scripts in
// math/, transformed with [sos, gain] = tf2sos(B, A). The a1 and a2 columns
// are the negatives of the tf2sos output.
// See https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
// -----------------------------------------------------------------------------

// TDK/InvenSense INMP441 equaliser, flattens the microphone response.
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/INMP441.pdf
// B ~= [1.00198, -1.99085, 0.98892]
// A ~= [1.0, -1.99518, 0.99518]
const SOS_Coefficients kInmp441Sos[] = {
    // {b1, b2, -a1, -a2}
    {-1.986920458344451f, +0.986963226946616f, +1.995178510504166f, -0.995184322194091f},
};
SOS_IIR_Filter INMP441(1.00197834654696f, kInmp441Sos);

// A-weighting, Fs = 48KHz. By Dr. Matt L., source: https://dsp.stackexchange.com/a/36122
const SOS_Coefficients kAWeightingSos[] = {
    {-2.00026996133106f, +1.00027056142719f, -1.060868438509278f, -0.163987445885926f},
    {+4.35912384203144f, +3.09120265783884f, +1.208419926363593f, -0.273166998428332f},
    {-0.70930303489759f, -0.29071868393580f, +1.982242159753048f, -0.982298594928989f},
};
SOS_IIR_Filter A_weighting(0.169994948147430f, kAWeightingSos);

// C-weighting, Fs = 48KHz. Designed by invfreqz curve fitting, see math/c_weighting.m
const SOS_Coefficients kCWeightingSos[] = {
    {+1.4604385758204708f, +0.5275070373815286f, +1.9946144559930252f, -0.9946217070140883f},
    {+0.2376222404939509f, +0.0140411206016894f, -1.3396585608422749f, -0.4421457807694559f},
    {-2.0000000000000000f, +1.0000000000000000f, +0.3775800047420818f, -0.0356365756680430f},
};
SOS_IIR_Filter C_weighting(-0.491647169337140f, kCWeightingSos);

// Referenced when MIC_EQUALIZER or MIC_WEIGHTING is set to 'None'.
No_IIR_Filter None;

// -----------------------------------------------------------------------------
// Sampling
// -----------------------------------------------------------------------------

// Amplitude the microphone reports at MIC_REF_DB, derived from its sensitivity.
constexpr double kMicRefAmpl = pow(10, MIC_SENSITIVITY / 20) * ((1 << (MIC_BITS - 1)) - 1);

using sample_t = int32_t;

// Shift the valid bits of an I2S frame down to the bottom of the word.
inline float micConvert(sample_t sample) { return sample >> (SAMPLE_BITS - MIC_BITS); }

constexpr i2s_port_t kI2sPort = I2S_NUM_0;
constexpr int kDmaBanks = 32;
constexpr int kDmaBankSize = SAMPLES_PER_BLOCK / 16;

// Handed from the sampling task to read() - one entry per sample block.
struct BlockSums {
  float sumSqrSpl;       // sum of squares after equalisation, unweighted
  float sumSqrWeighted;  // sum of squares after A/C weighting
};

QueueHandle_t blockQueue = nullptr;

// Sample buffer, reused in place: i2s_read fills it with integers, which are
// then converted to floats of the same width and filtered back over themselves.
float samples[SAMPLES_PER_BLOCK] __attribute__((aligned(4)));

// FreeRTOS priority and stack size (in 32-bit words).
constexpr UBaseType_t kTaskPriority = 4;
constexpr uint32_t kTaskStack = 2048;

bool i2sInit() {
  const i2s_config_t config = {
    mode : i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate : SAMPLE_RATE,
    bits_per_sample : i2s_bits_per_sample_t(SAMPLE_BITS),
    // NOTE: the 1.0.2 -> 1.0.3 arduino-esp32 update swapped ONLY_LEFT and
    // ONLY_RIGHT. If the readings sit at the noise floor, try the other one.
    channel_format : I2S_CHANNEL_FMT_ONLY_RIGHT,
    // Philips I2S. The old (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB)
    // pair is deprecated and both members are 0x01, so this is the same value.
    communication_format : I2S_COMM_FORMAT_STAND_I2S,
    intr_alloc_flags : ESP_INTR_FLAG_LEVEL1,
    dma_buf_count : kDmaBanks,
    dma_buf_len : kDmaBankSize,
    use_apll : true,  // APLL keeps the sample rate exact and independent of the CPU clock
    tx_desc_auto_clear : false,
    fixed_mclk : 0,
    mclk_multiple : I2S_MCLK_MULTIPLE_DEFAULT,
    bits_per_chan : I2S_BITS_PER_CHAN_DEFAULT
  };

  const i2s_pin_config_t pins = {
    // MCLK must be set explicitly. Leaving it out zero-initialises it to 0,
    // and 0 is a legal MCLK pin on the ESP32 (only 0, 1 and 3 are), so the
    // driver would repurpose GPIO0 - the boot strapping pin - as a clock
    // output. The microphone is a slave and needs no MCLK.
    mck_io_num : I2S_PIN_NO_CHANGE,
    bck_io_num : PIN_I2S_SCK,
    ws_io_num : PIN_I2S_WS,
    data_out_num : I2S_PIN_NO_CHANGE,
    data_in_num : PIN_I2S_SD
  };

  esp_err_t err = i2s_driver_install(kI2sPort, &config, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("sound_level: i2s_driver_install failed (%d)\n", err);
    return false;
  }

  err = i2s_set_pin(kI2sPort, &pins);
  if (err != ESP_OK) {
    Serial.printf("sound_level: i2s_set_pin failed (%d)\n", err);
    i2s_driver_uninstall(kI2sPort);
    return false;
  }
  return true;
}

// Reads blocks from I2S, filters them, and pushes the sums to blockQueue.
// Deliberately does the minimum per block: everything that needs a division
// or a logarithm is left to read().
void samplingTask(void* /*parameter*/) {
  size_t bytesRead = 0;

  // Discard the first block - the microphone needs time to start up
  // (up to 83ms on the INMP441).
  i2s_read(kI2sPort, &samples, sizeof(samples), &bytesRead, portMAX_DELAY);

  while (true) {
    // Blocks until the driver ISR has moved a full block out of the DMA
    // buffers. i2s_read writes raw integers into the float array; the
    // conversion below fixes that up in place.
    i2s_read(kI2sPort, &samples, sizeof(samples), &bytesRead, portMAX_DELAY);

    sample_t* intSamples = reinterpret_cast<sample_t*>(&samples);
    for (uint32_t i = 0; i < SAMPLES_PER_BLOCK; i++) samples[i] = micConvert(intSamples[i]);

    BlockSums block;
    block.sumSqrSpl = MIC_EQUALIZER.filter(samples, samples, SAMPLES_PER_BLOCK);
    block.sumSqrWeighted = MIC_WEIGHTING.filter(samples, samples, SAMPLES_PER_BLOCK);

    // Drop the oldest block rather than stalling the sampler if the main loop
    // has fallen behind - a late measurement is worth less than a current one.
    if (xQueueSend(blockQueue, &block, 0) != pdTRUE) {
      BlockSums discarded;
      xQueueReceive(blockQueue, &discarded, 0);
      xQueueSend(blockQueue, &block, 0);
    }
  }
}

// State of the measurement period currently being accumulated.
double leqSumSqr = 0;
uint32_t leqBlocks = 0;
bool sawOverload = false;
bool sawSignal = false;

// Sound pressure level of a sum of squares, in dB relative to the microphone
// reference amplitude. Doubles throughout: floats here would drag the task
// onto whichever core the FPU state happens to live on.
double toDb(double sumSqr, uint32_t sampleCount) {
  const double rms = sqrt(sumSqr / sampleCount);
  return MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(rms / kMicRefAmpl);
}

void resetPeriod() {
  leqSumSqr = 0;
  leqBlocks = 0;
  sawOverload = false;
  sawSignal = false;
}

}  // namespace

bool begin() {
  blockQueue = xQueueCreate(8, sizeof(BlockSums));
  if (blockQueue == nullptr) {
    Serial.println(F("sound_level: could not allocate the block queue"));
    return false;
  }

  if (!i2sInit()) return false;

  // NOTE: the task uses hardware FPU instructions, so ESP-IDF pins it to
  // whichever core it first runs on. Use xTaskCreatePinnedToCore to choose.
  if (xTaskCreate(samplingTask, "deciLight mic", kTaskStack, nullptr, kTaskPriority, nullptr) !=
      pdPASS) {
    Serial.println(F("sound_level: could not start the sampling task"));
    return false;
  }
  return true;
}

bool read(Reading& out, uint32_t timeoutMs) {
  const TickType_t timeout = pdMS_TO_TICKS(timeoutMs);
  BlockSums block;

  while (xQueueReceive(blockQueue, &block, timeout) == pdTRUE) {
    // Overload and noise floor are judged per block. Averaged over a whole
    // period a brief clip would disappear, which is exactly the case worth
    // reporting.
    const double blockDb = toDb(block.sumSqrSpl, SAMPLES_PER_BLOCK);
    if (blockDb > MIC_OVERLOAD_DB) sawOverload = true;
    if (!isnan(blockDb) && blockDb >= MIC_NOISE_DB) sawSignal = true;

    leqSumSqr += block.sumSqrWeighted;
    leqBlocks++;
    if (leqBlocks < LEQ_BLOCK_COUNT) continue;

    double leqDb = toDb(leqSumSqr, leqBlocks * SAMPLES_PER_BLOCK);

    if (sawOverload) {
      out.quality = Quality::Overload;
      out.leqDb = MIC_OVERLOAD_DB;
    } else if (!sawSignal || isnan(leqDb) || !isfinite(leqDb)) {
      out.quality = Quality::BelowNoiseFloor;
      out.leqDb = MIC_NOISE_DB;
    } else {
      out.quality = Quality::Ok;
      // Clamp so callers never have to reason about out-of-range values.
      if (leqDb < MIC_NOISE_DB) leqDb = MIC_NOISE_DB;
      if (leqDb > MIC_OVERLOAD_DB) leqDb = MIC_OVERLOAD_DB;
      out.leqDb = static_cast<float>(leqDb);
    }

    resetPeriod();
    return true;
  }

  return false;
}

}  // namespace sound_level
