#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "firmware/common/RPCanUtils.h"

static int test_intel_basic(void) {
  RPCanUtilsFrame frame = {
    .data = {0x34, 0x12, 0, 0, 0, 0, 0, 0},
    .dlc = 2,
  };
  uint32_t raw = rpcan_extract_raw(&frame, 0, 16, 0);
  if (raw != 0x1234U) {
    printf("test_intel_basic failed: got 0x%lX\n", (unsigned long)raw);
  }
  return (raw == 0x1234U) ? 0 : 1;
}

static int test_motorola_byte(void) {
  RPCanUtilsFrame frame = {
    .data = {0xAB, 0xCD, 0, 0, 0, 0, 0, 0},
    .dlc = 2,
  };
  uint32_t raw = rpcan_extract_raw(&frame, 7, 8, 1);
  if (raw != 0xABU) {
    printf("test_motorola_byte failed: got 0x%lX\n", (unsigned long)raw);
  }
  return (raw == 0xABU) ? 0 : 1;
}

static int test_motorola_cross(void) {
  RPCanUtilsFrame frame = {
    .data = {0x12, 0x34, 0x56, 0, 0, 0, 0, 0},
    .dlc = 3,
  };
  uint32_t raw = rpcan_extract_raw(&frame, 15, 12, 1);
  if (raw != 0x345U) {
    printf("test_motorola_cross failed: got 0x%lX\n", (unsigned long)raw);
  }
  return (raw == 0x345U) ? 0 : 1;
}

static int test_signed_scaling(void) {
  RPCanUtilsFrame frame = {
    .data = {0xF0, 0, 0, 0, 0, 0, 0, 0},
    .dlc = 1,
  };
  float val = rpcan_extract_scaled(&frame, 0, 8, 0, 1, 0.5f, -1.0f);
  if (!(val > -9.1f && val < -8.9f)) {
    printf("test_signed_scaling failed: got %.3f\n", (double)val);
  }
  return (val > -9.1f && val < -8.9f) ? 0 : 1;
}

static int test_motorola_known_case(void) {
  RPCanUtilsFrame frame = {
    .data = {0x1A, 0x5C, 0x34, 0xAC, 0x68, 0x68, 0xD0, 0x71},
    .dlc = 8,
  };
  float val = rpcan_extract_scaled(&frame, 7, 15, 1, 0, 0.01f, 0.0f);
  if (!(val > 33.739f && val < 33.741f)) {
    printf("test_motorola_known_case failed: got %.3f\n", (double)val);
  }
  return (val > 33.739f && val < 33.741f) ? 0 : 1;
}

static void print_usage(const char *prog) {
  printf("Usage:\n");
  printf("  %s                     # run built-in sanity tests\n", prog);
  printf("  %s <bytes..> --start N --len N --endian <intel|motorola|0|1> --signed <0|1> --factor F --offset O\n", prog);
  printf("Example:\n");
  printf("  %s 1A 5C 34 AC 68 68 D0 71 --start 7 --len 15 --endian motorola --signed 0 --factor 0.01 --offset 0\n", prog);
}

static int parse_hex_byte(const char *s, uint8_t *out) {
  char *end = NULL;
  unsigned long val = strtoul(s, &end, 16);
  if (end == s || *end != '\0' || val > 0xFFUL) {
    return 0;
  }
  *out = (uint8_t)val;
  return 1;
}

int main(int argc, char **argv) {
  if (argc > 1) {
    RPCanUtilsFrame frame = {0};
    uint8_t start_bit = 0;
    uint8_t sig_len = 0;
    uint8_t endian = 0;
    uint8_t is_signed = 0;
    float factor = 1.0f;
    float offset = 0.0f;
    int have_start = 0;
    int have_len = 0;
    int have_endian = 0;
    int have_signed = 0;
    int have_factor = 0;
    int have_offset = 0;

    int argi = 1;
    while (argi < argc && argv[argi][0] != '-' && frame.dlc < 8) {
      uint8_t byte = 0;
      if (!parse_hex_byte(argv[argi], &byte)) {
        print_usage(argv[0]);
        return 2;
      }
      frame.data[frame.dlc++] = byte;
      argi++;
    }

    for (; argi < argc; argi++) {
      if (strcmp(argv[argi], "--start") == 0 && argi + 1 < argc) {
        start_bit = (uint8_t)strtoul(argv[++argi], NULL, 0);
        have_start = 1;
      } else if (strcmp(argv[argi], "--len") == 0 && argi + 1 < argc) {
        sig_len = (uint8_t)strtoul(argv[++argi], NULL, 0);
        have_len = 1;
      } else if (strcmp(argv[argi], "--endian") == 0 && argi + 1 < argc) {
        const char *val = argv[++argi];
        if (strcmp(val, "intel") == 0 || strcmp(val, "0") == 0) {
          endian = 0;
        } else if (strcmp(val, "motorola") == 0 || strcmp(val, "1") == 0) {
          endian = 1;
        } else {
          print_usage(argv[0]);
          return 2;
        }
        have_endian = 1;
      } else if (strcmp(argv[argi], "--signed") == 0 && argi + 1 < argc) {
        is_signed = (uint8_t)strtoul(argv[++argi], NULL, 0);
        have_signed = 1;
      } else if (strcmp(argv[argi], "--factor") == 0 && argi + 1 < argc) {
        factor = strtof(argv[++argi], NULL);
        have_factor = 1;
      } else if (strcmp(argv[argi], "--offset") == 0 && argi + 1 < argc) {
        offset = strtof(argv[++argi], NULL);
        have_offset = 1;
      } else {
        print_usage(argv[0]);
        return 2;
      }
    }

    if (frame.dlc == 0 || !have_start || !have_len || !have_endian || !have_signed ||
        !have_factor || !have_offset) {
      print_usage(argv[0]);
      return 2;
    }

    uint32_t raw = rpcan_extract_raw(&frame, start_bit, sig_len, endian);
    float val = rpcan_extract_scaled(&frame, start_bit, sig_len, endian, is_signed, factor, offset);
    printf("raw=0x%lX (%lu)\n", (unsigned long)raw, (unsigned long)raw);
    printf("scaled=%.6f\n", (double)val);
    return 0;
  }

  int failures = 0;

  failures += test_intel_basic();
  failures += test_motorola_byte();
  failures += test_motorola_cross();
  failures += test_signed_scaling();
  failures += test_motorola_known_case();

  if (failures != 0) {
    printf("RPCanUtils sanity failures: %d\n", failures);
    return 1;
  }

  printf("RPCanUtils sanity OK\n");
  return 0;
}
