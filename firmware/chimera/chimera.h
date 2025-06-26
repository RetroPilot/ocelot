// header file for chimera's gateway config wizardee
#include <stdint.h>
#include <stddef.h>

#define MAX_CONFIG_ENTRIES 16
#define FLASH_CONFIG_MAGIC 0xDEADBEEF
#define CONFIG_FLASH_SECTOR 5
#define CONFIG_FLASH_ADDR 0x08020000

typedef struct __attribute__((packed)) {
  uint32_t can_id;  
  uint8_t msg_len_bytes; // MSB
  uint8_t sig_type;     
  uint8_t shift_amt;
  uint8_t sig_len;
  uint8_t endian_type;   // 0=big, 1=little     
  int16_t scale_mult;    // scale * 10
  int32_t scale_offs;    // offset * 10
  uint8_t enabled;
} can_signal_config_t;

typedef struct __attribute__((packed)) {
  uint32_t magic;                      // To detect valid config
  can_signal_config_t entries[MAX_CONFIG_ENTRIES];
  uint32_t crc32;                      // Optional: for integrity
} config_block_t;

uint32_t crc32(const void *data, size_t len) {
  const uint8_t *bytes = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFF;

  for (size_t i = 0; i < len; i++) {
    crc ^= bytes[i];
    for (int j = 0; j < 8; j++) {
    crc = (crc >> 1) ^ (0xEDB88320U & -(crc & 1));
  }
}

return ~crc;
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

void flash_config_format(void) {
  config_block_t blank;
  memset(&blank, 0, sizeof(blank));
  blank.magic = FLASH_CONFIG_MAGIC;
  blank.crc32 = crc32(&blank.entries[0], sizeof(blank.entries));

  flash_unlock();
  flash_erase_sector(CONFIG_FLASH_SECTOR);

  for (uint32_t i = 0; i < sizeof(config_block_t); i += 4) {
    flash_write_word(CONFIG_FLASH_ADDR + i,
      *(uint32_t *)((uint8_t *)&blank + i));
  }

  flash_lock();
  puts("Formatted config block with zeroed entries.\n");
}
