/*
 * deciLight - A/C weighted sound level measurement
 *
 * Sampling and IIR filtering run in a dedicated high-priority task so the
 * main loop is free to drive the LEDs and service the remote. The task hands
 * over sums of squares; the comparatively expensive logarithms happen here,
 * on the caller's side.
 */

#ifndef DECILIGHT_SOUND_LEVEL_H
#define DECILIGHT_SOUND_LEVEL_H

#include <stdint.h>

namespace sound_level {

enum class Quality : uint8_t {
  Ok,               // level is within the microphone's usable range
  BelowNoiseFloor,  // quieter than the microphone can resolve
  Overload,         // louder than the acoustic overload point
};

struct Reading {
  // Equivalent continuous sound level over LEQ_PERIOD_MS. Always finite and
  // clamped to [MIC_NOISE_DB, MIC_OVERLOAD_DB], whatever the quality.
  float leqDb;
  Quality quality;
};

// Brings up I2S and starts the sampling task. False means the microphone
// could not be initialised and no readings will ever arrive.
bool begin();

// Waits up to timeoutMs for a complete measurement period. Returns false on
// timeout, leaving out untouched.
bool read(Reading& out, uint32_t timeoutMs);

}  // namespace sound_level

#endif  // DECILIGHT_SOUND_LEVEL_H
