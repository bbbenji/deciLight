#include <stdio.h>

#include "harness.h"

int g_failures = 0;

int main() {
  test_settings();
  test_signal_light();
  test_remote_control();
  test_display();
  test_group_sync();

  if (g_failures == 0) {
    printf("\nall checks passed\n");
    return 0;
  }
  printf("\n%d check(s) failed\n", g_failures);
  return 1;
}
