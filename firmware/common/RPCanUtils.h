#ifndef RPCAN_UTILS_H
#define RPCAN_UTILS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t data[8];
  uint8_t dlc;
} RPCanUtilsFrame;

static inline uint8_t rpcan_utils_clamp_dlc(uint8_t dlc) {
  return (dlc > 8U) ? 8U : dlc;
}

static inline uint8_t rpcan_extract_bit(const RPCanUtilsFrame *frame, uint8_t dlc, uint8_t bit_index) {
  if (bit_index >= 64U) {
    return 0U;
  }
  uint8_t byte_index = (uint8_t)(bit_index / 8U);
  if (byte_index >= dlc) {
    return 0U;
  }
  uint8_t bit_pos = (uint8_t)(bit_index % 8U);
  return (frame->data[byte_index] >> bit_pos) & 1U;
}

static inline uint32_t rpcan_extract_raw_intel(const RPCanUtilsFrame *frame, uint8_t start_bit, uint8_t sig_len) {
  if (sig_len == 0U || sig_len > 32U || start_bit >= 64U ||
      ((uint16_t)start_bit + (uint16_t)sig_len) > 64U) {
    return 0U;
  }

  uint8_t dlc = rpcan_utils_clamp_dlc(frame->dlc);
  uint32_t raw = 0U;

  for (uint8_t i = 0; i < sig_len; i++) {
    uint8_t bit_val = rpcan_extract_bit(frame, dlc, (uint8_t)(start_bit + i));
    raw |= ((uint32_t)bit_val) << i;
  }

  return raw;
}

static inline uint32_t rpcan_extract_raw_motorola(const RPCanUtilsFrame *frame, uint8_t start_bit, uint8_t sig_len) {
  if (sig_len == 0U || sig_len > 32U || start_bit >= 64U) {
    return 0U;
  }

  uint8_t dlc = rpcan_utils_clamp_dlc(frame->dlc);
  uint32_t raw = 0U;
  int16_t bit = start_bit;

  for (uint8_t i = 0; i < sig_len; i++) {
    uint8_t bit_val = 0U;
    if (bit >= 0 && bit < 64) {
      bit_val = rpcan_extract_bit(frame, dlc, (uint8_t)bit);
    }

    raw = (raw << 1U) | bit_val;

    if ((bit % 8) == 0) {
      bit = (int16_t)(bit + 15);
    } else {
      bit = (int16_t)(bit - 1);
    }
  }

  return raw;
}

static inline uint32_t rpcan_extract_raw(const RPCanUtilsFrame *frame, uint8_t start_bit, uint8_t sig_len, uint8_t endian_type) {
  if ((endian_type & 1U) == 0U) {
    return rpcan_extract_raw_intel(frame, start_bit, sig_len);
  }
  return rpcan_extract_raw_motorola(frame, start_bit, sig_len);
}

static inline float rpcan_extract_scaled(const RPCanUtilsFrame *frame, uint8_t start_bit, uint8_t sig_len, uint8_t endian_type, uint8_t is_signed, float scale_factor, float scale_offset) {
  if (sig_len == 0U || sig_len > 32U) {
    return 0.0f;
  }

  uint32_t raw32 = rpcan_extract_raw(frame, start_bit, sig_len, endian_type);
  float value;

  if (is_signed & 1U) {
    if (sig_len < 32U) {
      uint32_t mask = (1U << sig_len) - 1U;
      if (raw32 & (1U << (sig_len - 1U))) {
        raw32 |= ~mask;
      }
    }
    value = (float)((int32_t)raw32);
  } else {
    if (sig_len < 32U) {
      raw32 &= (1U << sig_len) - 1U;
    }
    value = (float)raw32;
  }

  return value * scale_factor + scale_offset;
}

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // RPCAN_UTILS_H
