/*
 * Display A-weighted sound level measured by I2S Microphone
 *
 * (c)2019 Ivan Kostoski
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Sketch samples audio data from I2S microphone, processes the data
 * with digital IIR filters and calculates A or C weighted Equivalent
 * Continuous Sound Level (Leq)
 *
 * I2S is setup to sample data at Fs=48000KHz (fixed value due to
 * design of digital IIR filters). Data is read from I2S queue
 * in 'sample blocks' (default 125ms block, equal to 6000 samples)
 * by 'i2s_reader_task', filtered trough two IIR filters (equalizer
 * and weighting), summed up and pushed into 'samples_queue' as
 * sum of squares of filtered samples. The main task then pulls data
 * from the queue and calculates decibel value relative to microphone
 * reference amplitude, derived from datasheet sensitivity dBFS
 * value, number of bits in I2S data, and the reference value for
 * which the sensitivity is specified (typically 94dB, pure sine
 * wave at 1KHz).
 *
 * Displays line on the small OLED screen with 'short' LAeq(125ms)
 * response and numeric LAeq(1sec) dB value from the signal RMS.
 */

//
// Configuration
//

int deciLight = 1;

// Define IR
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
const uint16_t kRecvPin = 4;
IRrecv irrecv(kRecvPin);
decode_results results;

// Define FastLED
#include <FastLED.h>
#define LED_TYPE NEOPIXEL
#define NUM_LEDS 7 // How many LEDs are attached to the Arduino?
#define DATA_PIN 2 // Which pin on the Arduino is connected to the LEDs?
int brightness = 255; // LED brightness, 0 (min) to 255 (max)
CRGB leds[NUM_LEDS];

// Define preferences
#include <Preferences.h>
Preferences preferences;
int dB_min_default = 40;
int dB_max_default = 60;

// Define mic
#include <driver/i2s.h>
#include "sos-iir-filter.h"
#define LEQ_PERIOD 0.5        // second(s)
#define WEIGHTING A_weighting // Also avaliable: 'C_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS "LAeq"      // customize based on above weighting used
#define DB_UNITS "dBA"        // customize based on above weighting used

// NOTE: Some microphones require at least DC-Blocker filter
#define MIC_EQUALIZER INMP441 // See below for defined IIR filters or set to 'None' to disable
#define MIC_OFFSET_DB 3.0103  // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

// Customize these values from microphone datasheet
#define MIC_SENSITIVITY -26   // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB 94.0       // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB 116.0 // dB - Acoustic overload point
#define MIC_NOISE_DB 29       // dB - Noise floor
#define MIC_BITS 24           // valid number of bits in I2S data
#define MIC_CONVERT(s) (s >> (SAMPLE_BITS - MIC_BITS))

// Calculate reference amplitude value at compile time
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY) / 20) * ((1 << (MIC_BITS - 1)) - 1);

//
// I2S pins - Can be routed to almost any (unused) ESP32 pin.
//            SD can be any pin, inlcuding input only pins (36-39).
//            SCK (i.e. BCLK) and WS (i.e. L/R CLK) must be output capable pins
//
// Below ones are just example for my board layout, put here the pins you will use
//
#define I2S_WS 15
#define I2S_SCK 14
#define I2S_SD 32

// I2S peripheral to use (0 or 1)
#define I2S_PORT I2S_NUM_0

//
// Equalizer IIR filters to flatten microphone frequency response
// See respective .m file for filter design. Fs = 48Khz.
//
// Filters are represented as Second-Order Sections cascade with assumption
// that b0 and a0 are equal to 1.0 and 'gain' is applied at the last step
// B and A coefficients were transformed with GNU Octave:
// [sos, gain] = tf2sos(B, A)
// See: https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
// NOTE: SOS matrix 'a1' and 'a2' coefficients are negatives of tf2sos output
//

// TDK/InvenSense INMP441
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/INMP441.pdf
// B ~= [1.00198, -1.99085, 0.98892]
// A ~= [1.0, -1.99518, 0.99518]
SOS_IIR_Filter INMP441 = {
  gain : 1.00197834654696,
  sos : {// Second-Order Sections {b1, b2, -a1, -a2}
         {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}}
};

//
// Weighting filters
//

//
// A-weighting IIR Filter, Fs = 48KHz
// (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
// B = [0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003]
// A = [1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968]
SOS_IIR_Filter A_weighting = {
  gain : 0.169994948147430,
  sos : {// Second-Order Sections {b1, b2, -a1, -a2}
         {-2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
         {+4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
         {-0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}}
};

//
// Sampling
//
#define SAMPLE_RATE 48000 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS 32    // bits
#define SAMPLE_T int32_t
#define SAMPLES_SHORT (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE (SAMPLES_SHORT / 16)
#define DMA_BANKS 32

// Data we push to 'samples_queue'
struct sum_queue_t
{
  // Sum of squares of mic samples, after Equalizer filter
  float sum_sqr_SPL;
  // Sum of squares of weighted mic samples
  float sum_sqr_weighted;
  // Debug only, FreeRTOS ticks we spent processing the I2S data
  uint32_t proc_ticks;
};
QueueHandle_t samples_queue;

// Static buffer for block of samples
float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

//
// I2S Microphone sampling setup
//
void mic_i2s_init() {
  // Setup I2S to sample mono channel for SAMPLE_RATE * SAMPLE_BITS
  // NOTE: Recent update to Arduino_esp32 (1.0.2 -> 1.0.3)
  //       seems to have swapped ONLY_LEFT and ONLY_RIGHT channels
  const i2s_config_t i2s_config = {
    mode : i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate : SAMPLE_RATE,
    bits_per_sample : i2s_bits_per_sample_t(SAMPLE_BITS),
    channel_format : I2S_CHANNEL_FMT_ONLY_RIGHT,
    communication_format : i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    intr_alloc_flags : ESP_INTR_FLAG_LEVEL1,
    dma_buf_count : DMA_BANKS,
    dma_buf_len : DMA_BANK_SIZE,
    use_apll : true,
    tx_desc_auto_clear : false,
    fixed_mclk : 0
  };
  // I2S pin mapping
  const i2s_pin_config_t pin_config = {
    bck_io_num : I2S_SCK,
    ws_io_num : I2S_WS,
    data_out_num : -1, // not used
    data_in_num : I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  i2s_set_pin(I2S_PORT, &pin_config);
}

//
// I2S Reader Task
//
// Rationale for separate task reading I2S is that IIR filter
// processing can be scheduled to different core on the ESP32
// while main task can do something else, like update the
// display in the example
//
// As this is intended to run as separate high-priority task,
// we only do the minimum required work with the I2S data
// until it is 'compressed' into sum of squares
//
// FreeRTOS priority and stack size (in 32-bit words)
#define I2S_TASK_PRI 4
#define I2S_TASK_STACK 2048
//
void mic_i2s_reader_task(void *parameter) {
  mic_i2s_init();

  // Discard first block, microphone may have startup time (i.e. INMP441 up to 83ms)
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    // Block and wait for microphone values from I2S
    //
    // Data is moved from DMA buffers to our 'samples' buffer by the driver ISR
    // and when there is requested ammount of data, task is unblocked
    //
    // Note: i2s_read does not care it is writing in float[] buffer, it will write
    //       integer values to the given address, as received from the hardware peripheral.
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

    TickType_t start_tick = xTaskGetTickCount();

    // Convert (including shifting) integer microphone values to floats,
    // using the same buffer (assumed sample size is same as size of float),
    // to save a bit of memory
    SAMPLE_T *int_samples = (SAMPLE_T *)&samples;
    for (int i = 0; i < SAMPLES_SHORT; i++)
      samples[i] = MIC_CONVERT(int_samples[i]);

    sum_queue_t q;
    // Apply equalization and calculate Z-weighted sum of squares,
    // writes filtered samples back to the same buffer.
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);

    // Apply weighting and calucate weigthed sum of squares
    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);

    // Debug only. Ticks we spent filtering and summing block of I2S data
    q.proc_ticks = xTaskGetTickCount() - start_tick;

    // Send the sums to FreeRTOS queue where main task will pick them up
    // and further calcualte decibel values (division, logarithms, etc...)
    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

//
// Setup and main loop
//
// Note: Use doubles, not floats, here unless you want to pin
//       the task to whichever core it happens to run on at the moment
//
void setup() {

  // LED setup
  delay(2000);
  FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setDither(false);
  FastLED.setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(brightness);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 420);
  set_max_power_indicator_LED(13);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  // If needed, now you can actually lower the CPU frquency,
  // i.e. if you want to (slightly) reduce ESP32 power consumption
  setCpuFrequencyMhz(80); // It should run as low as 80MHz

  Serial.begin(115200);
  delay(1000); // Safety

  // IR setup
  irrecv.enableIRIn(); // Start the receiver
  while (!Serial)      // Wait for the serial connection to be establised.
    delay(50);

  // Create FreeRTOS queue
  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));

  // Create the I2S reader FreeRTOS task
  // NOTE: Current version of ESP-IDF will pin the task
  //       automatically to the first core it happens to run on
  //       (due to using the hardware FPU instructions).
  //       For manual control see: xTaskCreatePinnedToCore
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);


}

void loop() {

  sum_queue_t q;
  uint32_t Leq_samples = 0;
  double Leq_sum_sqr = 0;
  double Leq_dB = 0;

  // Read sum of samaples, calculated by 'i2s_reader_task'
  while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {

    preferences.begin("traffic", false);
    // Preferences setup
    unsigned int dB_min = preferences.getUInt("dB_min", dB_min_default);
    unsigned int dB_max = preferences.getUInt("dB_max", dB_max_default);

    if (irrecv.decode(&results)) {
      switch (results.value) {
        case 0xF700FF:
          // Serial.println("Bright+");
          if (brightness < 204) {
            brightness = brightness + 51;
          } else {
            brightness = 255;
          }
          FastLED.setBrightness(brightness);
          FastLED.show();
          break;
        case 0xF7807F:
          // Serial.println("Bright-");
          if (brightness > 51) {
            brightness = brightness - 51;
          } else {
            brightness = 10;
          }
          FastLED.setBrightness(brightness);
          FastLED.show();
          break;
        case 0xF740BF:
          // Serial.println("Off");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Black);
          FastLED.show();
          break;
        case 0xF7C03F:
          // Serial.println("On");
          deciLight = 1;
          break;
        case 0xF720DF:
          // Serial.println("Red");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Red);
          FastLED.show();
          break;
        case 0xF7A05F:
          // Serial.println("Green");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Green);
          FastLED.show();
          break;
        case 0xF7609F:
          // Serial.println("Blue");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Blue);
          FastLED.show();
          break;
        case 0xF7E01F:
          // Serial.println("White");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::White);
          FastLED.show();
          break;
        case 0xF710EF:
          // Serial.println("Tomato");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Tomato);
          FastLED.show();
          break;
        case 0xF7906F:
          // Serial.println("LightGreen");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::LightGreen);
          FastLED.show();
          break;
        case 0xF750AF:
          // Serial.println("SkyBlue");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::SkyBlue);
          FastLED.show();
          break;
        case 0xF7D02F:
          // Serial.println("Flash");
          dB_min++;
          if (deciLight == 1) {
            preferences.putUInt("dB_min", dB_min);
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
          }
          break;
        case 0xF730CF:
          // Serial.println("OrangeRed");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::OrangeRed);
          FastLED.show();
          break;
        case 0xF7B04F:
          // Serial.println("Cyan");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Cyan);
          FastLED.show();
          break;
        case 0xF7708F:
          // Serial.println("RebeccaPurple");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Purple);
          FastLED.show();
          break;
        case 0xF7F00F:
          // Serial.println("Strobe");
          if (deciLight == 1) {
            dB_min--;
            preferences.putUInt("dB_min", dB_min);
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
          }
          break;
        case 0xF708F7:
          // Serial.println("Orange");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Orange);
          FastLED.show();
          break;
        case 0xF78877:
          // Serial.println("Turquoise");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Turquoise);
          FastLED.show();
          break;
        case 0xF748B7:
          // Serial.println("Purple");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::MediumPurple);
          FastLED.show();
          break;
        case 0xF7C837:
          // Serial.println("Fade");
          if (deciLight == 1) {
            dB_max++;
            preferences.putUInt("dB_max", dB_max);
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
          }
          break;
        case 0xF728D7:
          // Serial.println("Yellow");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Yellow);
          FastLED.show();
          break;
        case 0xF7A857:
          // Serial.println("DarkCyan");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::DarkCyan);
          FastLED.show();
          break;
        case 0xF76897:
          // Serial.println("Plum");
          deciLight = 0;
          fill_solid(leds, NUM_LEDS, CRGB::Plum);
          FastLED.show();
          break;
        case 0xF7E817:
          // Serial.println("Smooth");
          if (deciLight == 1) {
            dB_max--;
            preferences.putUInt("dB_max", dB_max);
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
          }
          break;
        default:
          Serial.println("Unknown button pressed");
          Serial.print(results.value, HEX);
          Serial.println("");
      }
      irrecv.resume(); // Receive the next value
    }
    delay(100);
    preferences.end();

    // Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    // In case of acoustic overload or below noise floor measurement, report infinty Leq value
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    }
    else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    // Accumulate Leq sum
    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;

    // When we gather enough samples, calculate new Leq value
    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;

      // Serial output, customize (or remove) as needed
      Serial.printf("Current dB value: %.1f\n", Leq_dB);

      // Debug only
      // Serial.printf("%u processing ticks\n", q.proc_ticks);

      if (deciLight == 1) {
        if (Leq_dB < dB_min) {
          fill_solid(leds, NUM_LEDS, CRGB::Green);
          FastLED.show();
        }
        else if (Leq_dB < dB_max) {
          fill_solid(leds, NUM_LEDS, CRGB::Yellow);
          FastLED.show();
        }
        else {
          fill_solid(leds, NUM_LEDS, CRGB::Red);
          FastLED.show();
        }
      }

    }

  }
}