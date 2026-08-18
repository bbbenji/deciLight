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

  CASE("the radio channel is reported for diagnosis");
  startGroup("classroom");
  fakes::setChannel(6);
  CHECK(group_sync::channel() == 6, "channel %u, want 6", group_sync::channel());
}
