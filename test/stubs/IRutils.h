/*
 * Host stub: IRremoteESP8266 formatting helpers
 */

#ifndef DECILIGHT_TEST_IRUTILS_H
#define DECILIGHT_TEST_IRUTILS_H

#include <IRrecv.h>

String typeToString(const decode_type_t protocol, const bool isRepeat = false);
String resultToHexidecimal(const decode_results* const result);

#endif  // DECILIGHT_TEST_IRUTILS_H
