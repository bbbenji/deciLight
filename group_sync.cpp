#include "group_sync.h"

#if FEATURE_ESPNOW

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

#include "settings.h"
#include "signal_light.h"

namespace group_sync {
namespace {

// Broadcast rather than unicast: no discovery, no peer list to keep in sync
// across reboots, and no six-peer encryption cap. The group name in every
// message is what separates one set of lights from another.
const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum MessageType : uint8_t {
  MSG_LEVEL = 1,
  MSG_SETTINGS = 2,
  MSG_MODE = 3,
  MSG_SELF_TEST = 4,
};

// Packed and fixed-width so the format does not depend on how a particular
// compiler lays out a struct. Well inside ESP-NOW's 250-byte limit.
struct __attribute__((packed)) Message {
  uint8_t magic[2];  // 'd', 'L'
  uint8_t version;
  uint8_t type;
  uint32_t group;  // hash of the group name
  // A union rather than variable-length payloads, so every message is the
  // same size and a wrong length is enough to reject a foreign packet.
  union {
    struct {
      int16_t centiDb;
    } level;
    struct {
      uint8_t dbMin;
      uint8_t dbMax;
      uint8_t brightness;
    } settings;
    struct {
      uint8_t mode;  // signal_light::Mode
      uint8_t r, g, b;
    } mode;
  } body;
};

struct Peer {
  uint8_t mac[6];
  float levelDb;
  uint32_t heardMs;
  bool used;
};

bool running = false;
uint32_t groupId = 0;
uint32_t lastSentMs = 0;

// What actually prevents an applied change echoing around the group is the
// shape of the code: publishing happens only where a change originates -
// remote_control and web_control - and the apply path below calls settings
// and signal_light directly, never a publish. This flag is a second line of
// defence for the day someone wires publishing into one of those setters. It
// is deliberately not load-bearing today, and a test cannot reach it.
bool applyingRemote = false;

// Commands land here from the radio callback and are applied by tick() on the
// main thread. Only the most recent of each kind is kept: they are absolute
// states, not increments, so an older one has nothing to contribute.
struct Inbox {
  bool haveSettings;
  uint8_t dbMin, dbMax, brightness;
  bool haveMode;
  uint8_t mode;
  uint32_t color;
  bool haveSelfTest;
};
Inbox inbox = {};

// Written by the receive callback, which runs from the WiFi task. Only ever
// touched under this lock; the table is small enough that holding it for a
// linear scan costs nothing.
portMUX_TYPE peerLock = portMUX_INITIALIZER_UNLOCKED;
Peer peers[GROUP_MAX_PEERS] = {};

// FNV-1a. Any stable hash would do; this one is four lines and has no
// dependencies. Collisions would merge two groups, which is a cosmetic
// problem for a classroom light rather than a correctness one.
uint32_t hashGroup(const char* name) {
  uint32_t h = 2166136261u;
  for (const char* c = name; *c != '\0'; c++) {
    h ^= uint8_t(*c);
    h *= 16777619u;
  }
  return h;
}

bool sameMac(const uint8_t* a, const uint8_t* b) { return memcmp(a, b, 6) == 0; }

// Common header, plus the two conditions under which nothing may be sent.
bool fill(Message& msg, uint8_t type) {
  if (!running || applyingRemote) return false;
  msg.magic[0] = 'd';
  msg.magic[1] = 'L';
  msg.version = GROUP_PROTOCOL_VERSION;
  msg.type = type;
  msg.group = groupId;
  memset(&msg.body, 0, sizeof(msg.body));
  return true;
}

void send(const Message& msg) {
  esp_now_send(kBroadcast, reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != int(sizeof(Message))) return;

  Message msg;
  memcpy(&msg, data, sizeof(msg));
  if (msg.magic[0] != 'd' || msg.magic[1] != 'L') return;
  if (msg.version != GROUP_PROTOCOL_VERSION) return;
  if (msg.group != groupId) return;
  const uint32_t now = millis();

  if (msg.type != MSG_LEVEL) {
    // Everything else is a command. Record it and get out; applying it here
    // would mean touching settings and the LEDs from the WiFi task.
    portENTER_CRITICAL(&peerLock);
    switch (msg.type) {
      case MSG_SETTINGS:
        inbox.dbMin = msg.body.settings.dbMin;
        inbox.dbMax = msg.body.settings.dbMax;
        inbox.brightness = msg.body.settings.brightness;
        inbox.haveSettings = true;
        break;
      case MSG_MODE:
        inbox.mode = msg.body.mode.mode;
        inbox.color = (uint32_t(msg.body.mode.r) << 16) | (uint32_t(msg.body.mode.g) << 8) |
                      msg.body.mode.b;
        inbox.haveMode = true;
        break;
      case MSG_SELF_TEST:
        inbox.haveSelfTest = true;
        break;
      default:
        break;
    }
    portEXIT_CRITICAL(&peerLock);
    return;
  }

  const float level = float(msg.body.level.centiDb) / 100.0f;

  portENTER_CRITICAL(&peerLock);
  int slot = -1;
  int oldest = -1;
  for (int i = 0; i < GROUP_MAX_PEERS; i++) {
    if (peers[i].used && sameMac(peers[i].mac, mac)) {
      slot = i;
      break;
    }
    if (!peers[i].used && slot < 0) slot = i;
    if (peers[i].used && (oldest < 0 || peers[i].heardMs < peers[oldest].heardMs)) oldest = i;
  }
  // A full table evicts whoever has been quiet longest, so a group larger
  // than the cap degrades to the most recently active members rather than
  // ignoring newcomers entirely.
  if (slot < 0) slot = oldest;
  if (slot >= 0) {
    memcpy(peers[slot].mac, mac, 6);
    peers[slot].levelDb = level;
    peers[slot].heardMs = now;
    peers[slot].used = true;
  }
  portEXIT_CRITICAL(&peerLock);
}

}  // namespace

bool begin() {
  running = false;
  lastSentMs = 0;
  groupId = hashGroup(settings::groupName());
  memset(peers, 0, sizeof(peers));

  if (settings::groupName()[0] == '\0') {
    Serial.println(F("group: no group name set, working alone"));
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("group: esp_now_init failed"));
    return false;
  }
  esp_now_register_recv_cb(onReceive);

  // The peer has to sit on whichever interface WiFi actually brought up, and
  // channel 0 means "whatever channel we are on", which keeps working if the
  // station reconnects to an access point on a different one.
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, kBroadcast, 6);
  peer.channel = 0;
  peer.encrypt = false;
  peer.ifidx = (WiFi.getMode() & WIFI_MODE_STA) ? WIFI_IF_STA : WIFI_IF_AP;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println(F("group: could not add the broadcast peer"));
    esp_now_deinit();
    return false;
  }

  running = true;
  Serial.printf("group: '%s' on channel %u\n", settings::groupName(), channel());
  return true;
}

bool active() { return running; }

uint8_t channel() {
  uint8_t primary = 0;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  return primary;
}

uint8_t peerCount() {
  uint8_t n = 0;
  portENTER_CRITICAL(&peerLock);
  for (int i = 0; i < GROUP_MAX_PEERS; i++) {
    if (peers[i].used) n++;
  }
  portEXIT_CRITICAL(&peerLock);
  return n;
}

uint32_t lastHeardMs() {
  uint32_t newest = 0;
  bool any = false;
  portENTER_CRITICAL(&peerLock);
  for (int i = 0; i < GROUP_MAX_PEERS; i++) {
    if (peers[i].used && (!any || peers[i].heardMs > newest)) {
      newest = peers[i].heardMs;
      any = true;
    }
  }
  portEXIT_CRITICAL(&peerLock);
  return any ? millis() - newest : 0;
}

void publishLevel(float leqDb) {
  Message msg;
  if (!fill(msg, MSG_LEVEL)) return;
  // Only the periodic level message is throttled; commands are rare and
  // should go out the moment they happen.
  if (millis() - lastSentMs < GROUP_BROADCAST_MS) return;
  lastSentMs = millis();
  msg.group = groupId;
  msg.body.level.centiDb = int16_t(leqDb * 100.0f);
  send(msg);
}

void publishSettings() {
  Message msg;
  if (!fill(msg, MSG_SETTINGS)) return;
  const Settings& s = settings::get();
  msg.body.settings.dbMin = s.dbMin;
  msg.body.settings.dbMax = s.dbMax;
  msg.body.settings.brightness = s.brightness;
  send(msg);
}

void publishMode() {
  Message msg;
  if (!fill(msg, MSG_MODE)) return;
  const uint32_t rgb = signal_light::manualColor();
  msg.body.mode.mode = uint8_t(signal_light::mode());
  msg.body.mode.r = (rgb >> 16) & 0xFF;
  msg.body.mode.g = (rgb >> 8) & 0xFF;
  msg.body.mode.b = rgb & 0xFF;
  send(msg);
}

void publishSelfTest() {
  Message msg;
  if (!fill(msg, MSG_SELF_TEST)) return;
  send(msg);
}

float groupLevel(float ownDb, uint8_t combine) {
  if (!running) return ownDb;

  float loudest = ownDb;
  float total = ownDb;
  uint8_t counted = 1;

  portENTER_CRITICAL(&peerLock);
  for (int i = 0; i < GROUP_MAX_PEERS; i++) {
    if (!peers[i].used) continue;
    if (peers[i].levelDb > loudest) loudest = peers[i].levelDb;
    total += peers[i].levelDb;
    counted++;
  }
  portEXIT_CRITICAL(&peerLock);

  // With nobody else talking both rules collapse to our own reading, so a
  // lone unit needs no special case at the call site.
  return combine == COMBINE_AVERAGE ? total / float(counted) : loudest;
}

void tick() {
  if (!running) return;
  const uint32_t now = millis();

  portENTER_CRITICAL(&peerLock);
  for (int i = 0; i < GROUP_MAX_PEERS; i++) {
    // Unsigned arithmetic, so this survives the millis() rollover.
    if (peers[i].used && now - peers[i].heardMs >= GROUP_PEER_TIMEOUT_MS) peers[i].used = false;
  }
  const Inbox pending = inbox;
  inbox = Inbox{};
  portEXIT_CRITICAL(&peerLock);

  if (!pending.haveSettings && !pending.haveMode && !pending.haveSelfTest) return;

  applyingRemote = true;

  if (pending.haveSettings) {
    settings::setDbMin(pending.dbMin);
    settings::setDbMax(pending.dbMax);
    settings::setBrightness(pending.brightness);
    signal_light::setBrightness(settings::get().brightness);
  }
  if (pending.haveMode) {
    switch (signal_light::Mode(pending.mode)) {
      case signal_light::Mode::Manual: signal_light::setManualColor(pending.color); break;
      case signal_light::Mode::Off:    signal_light::setMode(signal_light::Mode::Off); break;
      default:                         signal_light::setMode(signal_light::Mode::Auto); break;
    }
  }
  if (pending.haveSelfTest) signal_light::startSelfTest();

  applyingRemote = false;
}

}  // namespace group_sync

#endif  // FEATURE_ESPNOW
