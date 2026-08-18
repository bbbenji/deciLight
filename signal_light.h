/*
 * deciLight - the light itself
 *
 * Owns the LED state and decides which signal colour the measured level maps
 * to. Nothing here blocks: brief confirmation flashes expire on their own the
 * next time tick() runs.
 */

#ifndef DECILIGHT_SIGNAL_LIGHT_H
#define DECILIGHT_SIGNAL_LIGHT_H

#include <stdint.h>

namespace signal_light {

enum class Mode : uint8_t {
  Auto,    // colour follows the sound level
  Manual,  // colour was picked on the remote and is held
  Off,     // LEDs dark, measurement continues
};

// Which band of the threshold window the level is in. Tracked separately from
// the colour so hysteresis has something to hold on to.
enum class Zone : uint8_t { Unknown, Quiet, Warn, Loud };

void begin(uint8_t brightness);

Mode mode();
void setMode(Mode next);

// Current zone, and the colour being held in Manual mode. For status displays.
Zone zone();
uint32_t manualColor();

// Smoothed level behind the current zone, or NAN before the first
// measurement settles.
float smoothedLevel();

// Switches to Manual and holds the given 0xRRGGBB colour.
void setManualColor(uint32_t rgb);

void setBrightness(uint8_t brightness);

// Which zones this unit lights, as ZONE_MASK_* bits, and what it shows when
// the level lands in a zone it does not cover. A unit covering all three
// behaves as a lone light always has; a stacked one covers a single zone and
// falls back to inactiveLevel otherwise, which is what makes three units read
// as one traffic signal. An inactiveLevel of zero is a dark lamp.
void setZones(uint8_t mask, uint8_t inactiveLevel);

// Feeds a new measurement into the smoothing filter and, in Auto mode,
// updates the signal colour. Ignored in Manual and Off.
void updateLevel(float leqDb, uint8_t dbMin, uint8_t dbMax);

// Blanks the LEDs briefly to acknowledge a remote command.
void flashAck();

// Runs a fixed colour sequence so a freshly wired ring can be checked against
// what it is supposed to be showing. Red and green come first and adjacent on
// purpose: WS2812 rings ship in both GRB and RGB orderings, and if those two
// appear swapped the ring is wired the other way round - which otherwise
// presents as the light showing red in a quiet room, indistinguishable from a
// threshold problem. Only the ring is overridden - the mode carries on
// underneath, so the light resumes on its own when the sequence ends.
void startSelfTest();
bool selfTestRunning();

// The colour that should be showing right now, or "" when not testing.
const char* selfTestLabel();

// Call from loop(). Ends expired flashes and pushes pending changes to the
// LEDs; a no-op when nothing has changed.
void tick();

}  // namespace signal_light

#endif  // DECILIGHT_SIGNAL_LIGHT_H
