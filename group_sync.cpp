#include "group_sync.h"

#if FEATURE_ESPNOW

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

#include "settings.h"

namespace group_sync {
namespace {

// Broadcast rather than unicast: no discovery, no peer list to keep in sync
// across reboots, and no six-peer encryption cap. The group name in every
// message is what separates one set of lights from another.
const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum MessageType : uint8_t {
  MSG_LEVEL = 1,
};

// Packed and fixed-width so the format does not depend on how a particular
// compiler lays out a struct. Well inside ESP-NOW's 250-byte limit.
struct __attribute__((packed)) Message {
  uint8_t magic[2];  // 'd', 'L'
  uint8_t version;
  uint8_t type;
  uint32_t group;    // hash of the group name
  int16_t levelCentiDb;
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

void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != int(sizeof(Message))) return;

  Message msg;
  memcpy(&msg, data, sizeof(msg));
  if (msg.magic[0] != 'd' || msg.magic[1] != 'L') return;
  if (msg.version != GROUP_PROTOCOL_VERSION) return;
  if (msg.group != groupId) return;
  if (msg.type != MSG_LEVEL) return;

  const uint32_t now = millis();
  const float level = float(msg.levelCentiDb) / 100.0f;

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
  if (!running) return;
  if (millis() - lastSentMs < GROUP_BROADCAST_MS) return;
  lastSentMs = millis();

  Message msg;
  msg.magic[0] = 'd';
  msg.magic[1] = 'L';
  msg.version = GROUP_PROTOCOL_VERSION;
  msg.type = MSG_LEVEL;
  msg.group = groupId;
  msg.levelCentiDb = int16_t(leqDb * 100.0f);

  esp_now_send(kBroadcast, reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
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
  portEXIT_CRITICAL(&peerLock);
}

}  // namespace group_sync

#endif  // FEATURE_ESPNOW
