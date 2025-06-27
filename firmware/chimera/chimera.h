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

const can_signal_config_t *steer_angle_major_cfg = NULL;
const can_signal_config_t *steer_angle_minor_cfg = NULL;
const can_signal_config_t *steer_angle_rate_cfg = NULL;
const can_signal_config_t *vehicle_speed_cfg = NULL;

// ********************** flash write **********************
void flash_unlock(void);
void flash_lock(void);
void flash_erase_sector(uint8_t sector);
void flash_write_word(uint32_t addr, uint32_t val);
void flash_config_format(void);
void handle_deferred_config_write(void);
uint32_t crc32(const void *data, size_t len);
bool validate_flash_config(const config_block_t *cfg);
void init_config_pointers(const config_block_t *cfg);

// Global/static variables to hold setup packet details for deferred processing
static USB_Setup_TypeDef last_setup_pkt;
static uint8_t *last_usb_data_ptr = NULL; // Points to the buffer provided by USB driver
volatile bool config_write_pending = false; // Flag to indicate a pending config write

// ############ functions ###############
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

// This function will handle the actual flash write.
// It should be called when EP0 OUT data transfer is complete.
void handle_deferred_config_write(void) {
  if (!config_write_pending) {
    return; // No pending write
  }

  // Clear the flag immediately to prevent re-entry
  config_write_pending = false;

  puts("Executing deferred config write.\n");

  if (last_setup_pkt.b.wLength.w == sizeof(can_signal_config_t) &&
      last_setup_pkt.b.wValue.w < MAX_CONFIG_ENTRIES &&
      last_usb_data_ptr != NULL) {

    can_signal_config_t *new_entry = (can_signal_config_t *)last_usb_data_ptr;
    config_block_t current;

    // Load existing config
    memcpy(&current, (void *)CONFIG_FLASH_ADDR, sizeof(config_block_t));

    // Update entry with the now-complete data
    current.entries[last_setup_pkt.b.wValue.w] = *new_entry;
    current.magic = FLASH_CONFIG_MAGIC;
    current.crc32 = crc32(&current.entries[0], sizeof(current.entries));

    // Write to flash
    flash_unlock();
    flash_erase_sector(CONFIG_FLASH_SECTOR);
    for (uint32_t i = 0; i < sizeof(config_block_t); i += 4) {
      flash_write_word(CONFIG_FLASH_ADDR + i,
                      *(uint32_t *)((uint8_t *)&current + i));
    }
    flash_lock();

    puts("Wrote config entry successfully.\n");

  }
  // Clear the usbdata buffer after the write is complete
  if (last_usb_data_ptr != NULL) {
    memset(last_usb_data_ptr, 0, last_setup_pkt.b.wLength.w);
    last_usb_data_ptr = NULL; // Invalidate pointer
  }
}

bool validate_flash_config(const config_block_t *cfg) {
  if (cfg->magic != FLASH_CONFIG_MAGIC ||
      cfg->crc32 != crc32(&cfg->entries[0], sizeof(cfg->entries))) {
    return false;
  }

  uint8_t sig_type_seen[16] = {0};

  for (int i = 0; i < MAX_CONFIG_ENTRIES; i++) {
    const can_signal_config_t *e = &cfg->entries[i];
    if (!e->enabled) continue;

    if (e->sig_type >= 16 || sig_type_seen[e->sig_type]) {
      return false;
    }

    sig_type_seen[e->sig_type] = 1;
  }

  return true;
}

void init_config_pointers(const config_block_t *cfg) {
  for (int i = 0; i < MAX_CONFIG_ENTRIES; i++) {
    const can_signal_config_t *e = &cfg->entries[i];
    if (!e->enabled) continue;

    switch (e->sig_type) {
      case 1: steer_angle_major_cfg = e; puts("steer angle major configured.\n"); break;
      case 2: steer_angle_minor_cfg = e; puts("steer angle minor configured.\n"); break;
      case 3: steer_angle_rate_cfg = e; puts("steer angle rate configured.\n"); break;
      case 4: vehicle_speed_cfg     = e; puts("vehicle speed configured.\n"); break;
      default: break;
    }
  }
}

