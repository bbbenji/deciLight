/*
 * deciLight - IR remote handling
 *
 * Decodes the 24-key NEC remote sold with cheap LED strips and turns key
 * presses into settings and signal_light calls. This is the only module that
 * knows what a button means.
 */

#ifndef DECILIGHT_REMOTE_CONTROL_H
#define DECILIGHT_REMOTE_CONTROL_H

namespace remote_control {

void begin();

// Call from loop(). Handles at most one key press per call and returns
// immediately when nothing has been received.
void poll();

}  // namespace remote_control

#endif  // DECILIGHT_REMOTE_CONTROL_H
