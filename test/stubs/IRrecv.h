/*
 * Host stub: IRremoteESP8266 receiver
 *
 * decode() pulls from a queue that tests fill with fake_ir_receive(), so a key
 * press can be simulated without any hardware or timing.
 */

#ifndef DECILIGHT_TEST_IRRECV_H
#define DECILIGHT_TEST_IRRECV_H

#include <IRremoteESP8266.h>

struct decode_results {
  decode_type_t decode_type = NEC;
  uint64_t value = 0;
};

class IRrecv {
 public:
  explicit IRrecv(uint16_t pin) : pin_(pin) {}
  void enableIRIn();
  bool decode(decode_results* results);
  void resume();

 private:
  uint16_t pin_;
};

#endif  // DECILIGHT_TEST_IRRECV_H
