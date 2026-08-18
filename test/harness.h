/*
 * A deliberately tiny test harness. No dependencies, no discovery magic:
 * suites are plain functions called from main.cpp.
 */

#ifndef DECILIGHT_TEST_HARNESS_H
#define DECILIGHT_TEST_HARNESS_H

#include <stdio.h>

extern int g_failures;

#define SUITE(name) printf("\n%s\n", name)
#define CASE(name) printf("  %s\n", name)

#define CHECK(cond, fmt, ...)                                    \
  do {                                                           \
    if (!(cond)) {                                               \
      printf("    FAIL %s:%d: " fmt "\n", __FILE__, __LINE__,    \
             ##__VA_ARGS__);                                     \
      g_failures++;                                              \
    }                                                            \
  } while (0)

void test_settings();
void test_signal_light();
void test_remote_control();
void test_display();

#endif  // DECILIGHT_TEST_HARNESS_H
