/*
 * deciLight - IR remote handling
 *
 * Decodes the 24-key NEC remote sold with cheap LED strips and turns key
 * presses into settings and signal_light calls. This is the only module that
 * knows what a button means.
 */

#ifndef DECILIGHT_REMOTE_CONTROL_H
#define DECILIGHT_REMOTE_CONTROL_H

#include <stdint.h>

namespace remote_control {

void begin();

// Call from loop(). Handles at most one key press per call and returns
// immediately when nothing has been received.
void poll();

// The most recent code received, for diagnostics. Lets a receiver be checked,
// and an unfamiliar remote be mapped, from the web interface instead of a
// serial cable. Hold-down repeats are not recorded - they would only overwrite
// the code that is actually of interest.
bool haveLastCode();
const char* lastCodeHex();    // "0xF700FF"
const char* lastProtocol();   // "NEC", "UNKNOWN", ...
bool lastCodeMapped();        // whether it is in the key map
uint32_t lastCodeAgeMs();

}  // namespace remote_control

#endif  // DECILIGHT_REMOTE_CONTROL_H
