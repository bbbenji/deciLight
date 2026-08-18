/*
 * deciLight - group synchronisation over ESP-NOW
 *
 * Units that share a radio channel and a group name act as one light. Each
 * broadcasts what its microphone is hearing a few times a second; each works
 * out the group's level for itself. There is no leader, no election and no
 * pairing step - joining a group means being given the same name.
 *
 * Two constraints are worth knowing before wiring this up:
 *
 *   - ESP-NOW only reaches peers on the same WiFi channel, and in station
 *     mode the channel belongs to whichever router was joined. Units on
 *     different networks will not hear each other however they are
 *     configured. channel() reports what this unit ended up on.
 *
 *   - Membership is a convention, not a secret. The transport is broadcast
 *     and the group name is a filter, so anything in range running this
 *     firmware with the same name joins in.
 */

#ifndef DECILIGHT_GROUP_SYNC_H
#define DECILIGHT_GROUP_SYNC_H

#include <stdint.h>

#include "config.h"

namespace group_sync {

#if FEATURE_ESPNOW

// Brings up ESP-NOW on the interface WiFi is already using. Must be called
// after networking is up, because the radio has to be started first. False
// means the group features are unavailable; the light works alone regardless.
bool begin();

// True once ESP-NOW is running and a group name is configured.
bool active();

// The radio channel this unit is on. Peers must match, so this is the first
// thing to check when a group will not form.
uint8_t channel();

// Peers heard from within GROUP_PEER_TIMEOUT_MS, excluding this unit.
uint8_t peerCount();

// Age in milliseconds of the most recently heard peer, or 0 when there are
// none. For a "last heard" readout.
uint32_t lastHeardMs();

// Offers this unit's own measurement to the group. Rate limited internally,
// so it is safe to call for every reading.
void publishLevel(float leqDb);

// The level the light should follow, combined from this unit and its live
// peers according to the configured rule. Returns ownDb unchanged when the
// group is inactive or nobody else is talking, so the caller never needs to
// special-case a lone unit.
float groupLevel(float ownDb, uint8_t combine);

// Relays a locally-originated change to the group. Each is a no-op while a
// received message is being applied, so a change can never echo back and
// forth between units.
//
// Thresholds and LED brightness are shared because they describe the room.
// Zone masks, group name, screen brightness and the inactive level are not:
// they describe an individual unit's place in the arrangement, and copying
// them would collapse a stack into three identical lights.
void publishSettings();
void publishMode();
void publishSelfTest();

// Call from loop(). Ages out peers that have gone quiet, and applies anything
// received since the last call.
//
// Messages are applied here rather than in the radio callback, which runs on
// the WiFi task: settings and LED state are not safe to touch from two
// threads, and NVS writes least of all.
void tick();

#else

inline bool begin() { return false; }
inline bool active() { return false; }
inline uint8_t channel() { return 0; }
inline uint8_t peerCount() { return 0; }
inline uint32_t lastHeardMs() { return 0; }
inline void publishLevel(float) {}
inline void publishSettings() {}
inline void publishMode() {}
inline void publishSelfTest() {}
inline float groupLevel(float ownDb, uint8_t) { return ownDb; }
inline void tick() {}

#endif  // FEATURE_ESPNOW

}  // namespace group_sync

#endif  // DECILIGHT_GROUP_SYNC_H
