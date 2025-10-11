#pragma once

#include <stdint.h>
#include <string.h>

#define TORQUE_LUT_MAGIC         0x2B00B1E5
#define TORQUE_LUT_FLASH_SECTOR  6
#define TORQUE_LUT_FLASH_ADDR    0x08040000
#define TORQUE_LUT_SIZE          256

// Struct stored in flash
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint8_t table[TORQUE_LUT_SIZE]; 
  uint32_t crc;
} torque_lut_block_t;

// RAM buffer of current LUT
static uint8_t torque_scale_table[TORQUE_LUT_SIZE];

// Default LUT generator
static void torque_lut_generate_default(void) {
  for (int i = 0; i < TORQUE_LUT_SIZE; i++) {
    if (i <= 100)
      torque_scale_table[i] = 100 - (i / 2);
    else
      torque_scale_table[i] = 50;
  }
}

static void torque_lut_save(void) {
  torque_lut_block_t block;
  block.magic = TORQUE_LUT_MAGIC;
  memcpy(block.table, torque_scale_table, TORQUE_LUT_SIZE);
  block.crc = crc32(&block.table[0], TORQUE_LUT_SIZE);

  flash_unlock();
  flash_erase_sector(TORQUE_LUT_FLASH_SECTOR);
  for (uint32_t i = 0; i < sizeof(block); i += 4) {
    flash_write_word(TORQUE_LUT_FLASH_ADDR + i,
      *(uint32_t *)((uint8_t *)&block + i));
  }
  flash_lock();
  puts("Saved torque LUT to flash.\n");
  NVIC_SystemReset();
}

// Load from flash or generate defaults if invalid
static void torque_lut_init(void) {
  const torque_lut_block_t *stored = (const torque_lut_block_t *)TORQUE_LUT_FLASH_ADDR;

  if (stored->magic == TORQUE_LUT_MAGIC &&
      crc32(&stored->table[0], TORQUE_LUT_SIZE) == stored->crc) {
    memcpy(torque_scale_table, stored->table, TORQUE_LUT_SIZE);
    puts("Loaded torque LUT from flash.\n");
  } else {
    puts("Torque LUT invalid or missing. Generating default.\n");
    torque_lut_generate_default();
    torque_lut_save();
  }
}

// Given speed in (km/h * 100), return torque scale percent (0–100)
static uint16_t get_torque_scale_percent(uint32_t vehicle_speed_can) {
  uint16_t kph = vehicle_speed_can / 100;
  if (kph > 255) kph = 255;
  return torque_scale_table[kph];
}