/*
 * Host stub: Preferences (NVS)
 *
 * Backed by an in-memory map that survives begin()/end() the way real flash
 * does, so tests can assert on how many writes a sequence of edits produces.
 */

#ifndef DECILIGHT_TEST_PREFERENCES_H
#define DECILIGHT_TEST_PREFERENCES_H

#include <Arduino.h>

class Preferences {
 public:
  bool begin(const char* name, bool readOnly = false);
  void end();
  uint32_t getUInt(const char* key, uint32_t defaultValue = 0);
  size_t putUInt(const char* key, uint32_t value);
  size_t getString(const char* key, char* out, size_t max);
  size_t putString(const char* key, const char* value);
};

#endif  // DECILIGHT_TEST_PREFERENCES_H
