/*
 * group_sync: the wire format, who gets listened to, and what happens on the
 * main thread rather than the radio callback.
 *
 * Packets are not hand-assembled here. A test asks the module to transmit,
 * captures what it produced, and hands that back as if a peer had sent it -
 * so encode and decode are exercised against each other and neither can
 * drift from a copy of the struct kept in the test.
 */

#include "harness.h"

#include <string.h>

#include "../config.h"
#include "../group_sync.h"
#include "../settings.h"
#include "../signal_light.h"
#include "fakes.h"

namespace {

// Byte offsets in the header. Part of the documented wire format, so pinning
// them is intentional: a change here is a protocol change.
constexpr int OFF_VERSION = 2;
constexpr int OFF_TYPE = 3;
constexpr int OFF_GROUP = 4;

const uint8_t kPeerA[6] = {0x02, 0, 0, 0, 0, 0xA1};
const uint8_t kPeerB[6] = {0x02, 0, 0, 0, 0, 0xB2};

uint8_t packet[64];
int packetLen = 0;

// Runs the module far enough to have transmitted something, and keeps it.
bool capture(void (*publish)()) {
  const int before = fakes::groupPacketsSent();
  publish();
  if (fakes::groupPacketsSent() == before) return false;
  packetLen = fakes::groupPacketLength();
  memcpy(packet, fakes::groupPacket(), size_t(packetLen));
  return true;
}

bool captureLevel(float db) {
  const int before = fakes::groupPacketsSent();
  group_sync::publishLevel(db);
  if (fakes::groupPacketsSent() == before) return false;
  packetLen = fakes::groupPacketLength();
  memcpy(packet, fakes::groupPacket(), size_t(packetLen));
  return true;
}

void startGroup(const char* name) {
  fakes::reset();
  settings::begin();
  signal_light::begin(LED_BRIGHTNESS_DEFAULT);
  settings::setGroupName(name);
  group_sync::begin();
}

}  // namespace

void test_group_sync() {
  SUITE("group_sync");

  CASE("a unit with no group name works alone and stays silent");
  startGroup("");
  CHECK(!group_sync::active(), "a nameless unit joined a group");
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  group_sync::publishLevel(50.0f);
  CHECK(fakes::groupPacketsSent() == 0, "a nameless unit transmitted");
  CHECK(group_sync::groupLevel(50.0f, COMBINE_LOUDEST) == 50.0f,
        "a nameless unit should follow its own level");

  CASE("a named unit joins and broadcasts its level");
  startGroup("classroom");
  CHECK(group_sync::active(), "a named unit did not start");
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  CHECK(captureLevel(52.0f), "no level was broadcast");
  CHECK(packet[OFF_VERSION] == GROUP_PROTOCOL_VERSION, "wrong protocol version on the wire");

  CASE("level broadcasts are rate limited");
  int before = fakes::groupPacketsSent();
  for (int i = 0; i < 20; i++) group_sync::publishLevel(52.0f);
  CHECK(fakes::groupPacketsSent() == before, "%d unthrottled broadcasts",
        fakes::groupPacketsSent() - before);
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  group_sync::publishLevel(52.0f);
  CHECK(fakes::groupPacketsSent() == before + 1, "throttle never released");

  CASE("a peer's level is heard and counted");
  startGroup("classroom");
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  CHECK(captureLevel(80.0f), "setup failed");
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);
  CHECK(group_sync::peerCount() == 1, "peer count %u, want 1", group_sync::peerCount());
  CHECK(group_sync::groupLevel(40.0f, COMBINE_LOUDEST) > 79.0f,
        "the peer's level did not reach the group");

  CASE("two peers, loudest wins");
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  CHECK(captureLevel(60.0f), "setup failed");
  fakes::deliverGroupPacket(kPeerB, packet, packetLen);
  CHECK(group_sync::peerCount() == 2, "peer count %u, want 2", group_sync::peerCount());
  const float loudest = group_sync::groupLevel(40.0f, COMBINE_LOUDEST);
  CHECK(loudest > 79.0f && loudest < 81.0f, "loudest gave %.1f, want about 80", loudest);

  CASE("the same peers, averaged");
  // 40 (ours) + 80 + 60 over three units.
  const float mean = group_sync::groupLevel(40.0f, COMBINE_AVERAGE);
  CHECK(mean > 59.0f && mean < 61.0f, "average gave %.1f, want about 60", mean);

  CASE("a peer that goes quiet stops holding the group");
  fakes::advanceMillis(GROUP_PEER_TIMEOUT_MS + 1);
  group_sync::tick();
  CHECK(group_sync::peerCount() == 0, "%u stale peer(s) survived", group_sync::peerCount());
  CHECK(group_sync::groupLevel(40.0f, COMBINE_LOUDEST) == 40.0f,
        "a departed peer is still driving the light");

  CASE("another group's traffic is ignored");
  startGroup("classroom");
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  CHECK(captureLevel(90.0f), "setup failed");
  packet[OFF_GROUP] ^= 0xFF;  // a different group name would hash differently
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);
  CHECK(group_sync::peerCount() == 0, "joined a foreign group");

  CASE("a different protocol version is ignored");
  CHECK(captureLevel(90.0f) || true, "");
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  CHECK(captureLevel(90.0f), "setup failed");
  packet[OFF_VERSION] = GROUP_PROTOCOL_VERSION + 1;
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);
  CHECK(group_sync::peerCount() == 0, "accepted a future protocol version");

  CASE("a truncated packet is ignored");
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  CHECK(captureLevel(90.0f), "setup failed");
  fakes::deliverGroupPacket(kPeerA, packet, packetLen - 1);
  CHECK(group_sync::peerCount() == 0, "accepted a short packet");

  // Settings and LED state are not safe to touch from the WiFi task, so
  // arrival must only record and tick() must do the work.
  CASE("a received setting is applied by tick(), not by the callback");
  startGroup("classroom");
  settings::setDbMin(41);
  settings::setDbMax(61);
  CHECK(capture(group_sync::publishSettings), "no settings message was sent");
  settings::setDbMin(50);
  settings::setDbMax(70);
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);
  CHECK(settings::get().dbMin == 50, "the callback applied the change itself");
  group_sync::tick();
  CHECK(settings::get().dbMin == 41 && settings::get().dbMax == 61,
        "tick() did not apply the received settings: %u/%u", settings::get().dbMin,
        settings::get().dbMax);

  // Without this property two units would trade the same change forever. The
  // received value differs from what this unit holds, so the apply genuinely
  // happens rather than being skipped as a no-op.
  CASE("applying a received change does not rebroadcast it");
  startGroup("classroom");
  settings::setDbMin(45);
  settings::setDbMax(65);
  CHECK(capture(group_sync::publishSettings), "setup failed");
  settings::setDbMin(35);
  settings::setDbMax(55);
  const int quiet = fakes::groupPacketsSent();
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);
  group_sync::tick();
  CHECK(settings::get().dbMin == 45, "precondition: the change was not applied");
  CHECK(fakes::groupPacketsSent() == quiet, "an applied change echoed back out (%d packet(s))",
        fakes::groupPacketsSent() - quiet);

  CASE("a received mode change reaches the light");
  startGroup("classroom");
  signal_light::setManualColor(0x123456);
  CHECK(capture(group_sync::publishMode), "no mode message was sent");
  signal_light::setMode(signal_light::Mode::Auto);
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);
  group_sync::tick();
  signal_light::tick();
  CHECK(signal_light::mode() == signal_light::Mode::Manual, "mode did not transfer");
  CHECK(signal_light::manualColor() == 0x123456u, "colour did not transfer: 0x%06X",
        signal_light::manualColor());

  CASE("a received self test starts the sequence");
  startGroup("classroom");
  CHECK(capture(group_sync::publishSelfTest), "no self-test message was sent");
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);
  group_sync::tick();
  CHECK(signal_light::selfTestRunning(), "the group self test did not start");

  CASE("a unit without a name falls back to its MAC suffix");
  startGroup("classroom");
  CHECK(strlen(group_sync::localName()) > 0, "no name at all");
  CHECK(strcmp(group_sync::localName(), "ABCD") == 0, "MAC fallback gave '%s', want ABCD",
        group_sync::localName());

  CASE("a configured name is used instead");
  fakes::reset();
  settings::begin();
  settings::setUnitName("top");
  settings::setGroupName("classroom");
  group_sync::begin();
  CHECK(strcmp(group_sync::localName(), "top") == 0, "name is '%s', want top",
        group_sync::localName());

  CASE("the roster reports each peer by name and level");
  startGroup("classroom");
  settings::setUnitName("alpha");
  group_sync::begin();
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  CHECK(captureLevel(63.0f), "setup failed");
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);

  group_sync::PeerInfo list[GROUP_MAX_PEERS];
  uint8_t n = group_sync::peerList(list, GROUP_MAX_PEERS);
  CHECK(n == 1, "roster has %u entries, want 1", n);
  CHECK(strcmp(list[0].name, "alpha") == 0, "roster name '%s', want alpha", list[0].name);
  CHECK(list[0].levelDb > 62.0f && list[0].levelDb < 64.0f, "roster level %.1f, want 63",
        list[0].levelDb);

  // A stack with a gap leaves a band unlit; one with an overlap lights two
  // lamps at once. Both look like faults rather than misconfiguration.
  CASE("zone gaps and overlaps across the group are detectable");
  startGroup("classroom");
  settings::setGroupLevel(true);

  settings::setZones(ZONE_MASK_QUIET);
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  CHECK(captureLevel(50.0f), "setup failed");
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);

  settings::setZones(ZONE_MASK_WARN);
  fakes::advanceMillis(GROUP_BROADCAST_MS + 1);
  CHECK(captureLevel(50.0f), "setup failed");
  fakes::deliverGroupPacket(kPeerB, packet, packetLen);

  // Ours is warn, peer A is quiet, peer B is warn: nothing covers loud, and
  // warn is covered twice.
  CHECK((group_sync::zoneCoverage() & ZONE_MASK_LOUD) == 0, "loud reported as covered");
  CHECK((group_sync::zoneCoverage() & ZONE_MASK_QUIET) != 0, "quiet reported as uncovered");
  CHECK((group_sync::zoneOverlap() & ZONE_MASK_WARN) != 0, "the warn overlap went unnoticed");
  CHECK((group_sync::zoneOverlap() & ZONE_MASK_QUIET) == 0, "quiet wrongly flagged as doubled");

  CASE("a unit running independently is left out of coverage");
  startGroup("classroom");
  settings::setGroupLevel(false);
  settings::setZones(ZONE_MASK_ALL);
  CHECK(group_sync::zoneCoverage() == 0, "an independent unit counted towards the stack");

  // Group-wide, unlike zones: two units disagreeing about it would quietly
  // show different colours from the same readings.
  CASE("the combine rule travels with the settings");
  startGroup("classroom");
  settings::setCombine(COMBINE_AVERAGE);
  CHECK(capture(group_sync::publishSettings), "no settings message was sent");
  settings::setCombine(COMBINE_LOUDEST);
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);
  group_sync::tick();
  CHECK(settings::get().combine == COMBINE_AVERAGE, "combine did not transfer: %u",
        settings::get().combine);

  // Broadcast is unacknowledged, so a one-shot command would be lost with the
  // packet that carried it.
  CASE("commands are repeated, paced rather than blocking");
  startGroup("classroom");
  int sent = fakes::groupPacketsSent();
  settings::setDbMin(47);
  group_sync::publishSettings();
  CHECK(fakes::groupPacketsSent() == sent + 1, "the first copy did not go out immediately");
  group_sync::tick();
  CHECK(fakes::groupPacketsSent() == sent + 1, "a repeat went out before its gap elapsed");
  for (int i = 1; i < GROUP_COMMAND_REPEATS; i++) {
    fakes::advanceMillis(GROUP_COMMAND_GAP_MS + 1);
    group_sync::tick();
  }
  CHECK(fakes::groupPacketsSent() == sent + GROUP_COMMAND_REPEATS,
        "%d copies sent, want %u", fakes::groupPacketsSent() - sent, GROUP_COMMAND_REPEATS);

  CASE("repeated self tests do not restart a running sequence");
  startGroup("classroom");
  CHECK(capture(group_sync::publishSelfTest), "setup failed");
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);
  group_sync::tick();
  signal_light::tick();
  CHECK(signal_light::selfTestRunning(), "the test did not start");
  const char* first = signal_light::selfTestLabel();
  fakes::advanceMillis(1500);
  signal_light::tick();
  CHECK(strcmp(signal_light::selfTestLabel(), first) != 0, "the sequence did not advance");
  // The repeats arrive while it is already running.
  fakes::deliverGroupPacket(kPeerA, packet, packetLen);
  group_sync::tick();
  signal_light::tick();
  CHECK(strcmp(signal_light::selfTestLabel(), first) != 0,
        "a repeated command restarted the sequence");

  CASE("the radio channel is reported for diagnosis");
  startGroup("classroom");
  fakes::setChannel(6);
  CHECK(group_sync::channel() == 6, "channel %u, want 6", group_sync::channel());
}
