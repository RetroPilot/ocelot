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

// CRC32
static uint32_t crc32(const void *data, uint32_t len) {
  const uint8_t *d = (const uint8_t *)data;
  uint32_t crc = ~0U;
  while (len--) {
    crc ^= *d++;
    for (int k = 0; k < 8; k++)
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

// Default LUT generator
static void torque_lut_generate_default(void) {
  for (int i = 0; i < TORQUE_LUT_SIZE; i++) {
    if (i <= 100)
      torque_scale_table[i] = 100 - (i / 2);
    else
      torque_scale_table[i] = 50;
  }
}

void flash_unlock(void) {
  __disable_irq();
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
  }
  __enable_irq();
}
  
void flash_lock(void) {
    __disable_irq();
  FLASH->CR |= FLASH_CR_LOCK;
  __enable_irq();
}

void flash_erase_sector(uint8_t sector) {
    __disable_irq();
  while (FLASH->SR & FLASH_SR_BSY);  // wait for not busy
  
  FLASH->CR &= ~(FLASH_CR_SNB | FLASH_CR_PSIZE);
  FLASH->CR |= FLASH_CR_PSIZE_1;                // 32-bit programming
  FLASH->CR |= FLASH_CR_SER | (sector << 3);    // select sector and set SER
  FLASH->CR |= FLASH_CR_STRT;                   // start erase
  
  while (FLASH->SR & FLASH_SR_BSY);  // wait until done
  FLASH->CR &= ~FLASH_CR_SER;        // clear SER
  
  puts("Erased flash sector ");
  puth(sector);
  puts("\n");
  __enable_irq();
}

void flash_write_word(uint32_t address, uint32_t data) {
    __disable_irq();
  while (FLASH->SR & FLASH_SR_BSY);  // wait
  
  FLASH->CR &= ~(FLASH_CR_PSIZE);
  FLASH->CR |= FLASH_CR_PSIZE_1;     // 32-bit programming
  FLASH->CR |= FLASH_CR_PG;
  
  *(volatile uint32_t *)address = data;
  
  while (FLASH->SR & FLASH_SR_BSY);  // wait
  FLASH->CR &= ~FLASH_CR_PG;
  
  // Debug
//   puts("Wrote flash at 0x");
//   puth(address);
//   puts(": 0x");
//   puth(data);
//   puts("\n");
  
  // Verify
  uint32_t verify = *(volatile uint32_t *)address;
  if (verify != data) {
    puts("VERIFY FAIL at 0x");
    puth(address);
    puts(": wrote 0x");
    puth(data);
    puts(", read 0x");
    puth(verify);
    puts("\n");
  }
  __enable_irq();
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
