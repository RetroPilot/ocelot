// header file for chimera's gateway config wizardee
#include <stdint.h>
#include <stddef.h>

#define MAX_CONFIG_ENTRIES 32
#define FLASH_CONFIG_MAGIC 0xDEADBEEF
#define CONFIG_FLASH_SECTOR 5
#define CONFIG_FLASH_ADDR 0x08020000

#define CFG_TYPE_SYS  1
#define CFG_TYPE_CAN  2
#define CFG_TYPE_ADC  3
#define CFG_TYPE_VSS  4
#define GFG_TYPE_GPIO 5

#include <stdint.h>
#include <stddef.h>

typedef struct __attribute__((packed, aligned(1))) {
  union {
    uint8_t raw[23];

    struct __attribute__((packed)) {
      uint8_t debug_lvl;
      uint8_t can_out_en;
      uint8_t iwdg_en;
      uint8_t cfg_extra[20];
    } sys;

    struct __attribute__((packed)) {
      uint32_t can_id;
      int32_t scale_offs;
      int16_t scale_mult;
      uint8_t msg_len_bytes;
      uint8_t sig_type;
      uint8_t shift_amt;
      uint8_t sig_len;
      uint8_t endian_type;
      uint8_t enabled;
      uint8_t is_signed;
      uint8_t extra[6];
    } can;

    struct __attribute__((packed)) {
      uint32_t adc1;
      uint32_t adc2;
      uint16_t adc_tolerance;
      uint8_t adc_num;
      uint8_t adc_reserved[4];
      uint8_t adc_en;
      uint8_t extra[7];
    } adc;

    struct __attribute__((packed)) {
      uint16_t vss_ppd;
      uint8_t is_km;
      uint8_t extra[20];
    } vss;

    // struct __attribute__((packed)) {
    //   uint8_t gpio_in;
    //   uint8_t gpio_out;
    //   uint8_t extra[21];
    // } gpio;

  };
  uint8_t cfg_type;
} flash_config_t;

_Static_assert(sizeof(flash_config_t) == 24, "flash_config_t must be exactly 24 bytes");

typedef struct __attribute__((packed)) {
  uint32_t magic;                      // To detect valid config
  flash_config_t entries[MAX_CONFIG_ENTRIES];
  uint32_t crc32;                      // Optional: for integrity
} config_block_t;

// CAN signal config pointers
const flash_config_t *signal_configs[MAX_CONFIG_ENTRIES] = { NULL };

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
  __disable_irq();
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
  __enable_irq();
}

// This function will handle the actual flash write.
// It should be called when EP0 OUT data transfer is complete.
void handle_deferred_config_write(void) {
  __disable_irq();
  if (!config_write_pending) {
    return; // No pending write
  }

  // Clear the flag immediately to prevent re-entry
  config_write_pending = false;

  puts("Executing deferred config write.\n");

  if (last_setup_pkt.b.wLength.w == sizeof(flash_config_t) &&
      last_setup_pkt.b.wValue.w < MAX_CONFIG_ENTRIES &&
      last_usb_data_ptr != NULL) {

    flash_config_t *new_entry = (flash_config_t *)last_usb_data_ptr;
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
  __enable_irq();
}

bool validate_flash_config(const config_block_t *cfg) {
  if (cfg->magic != FLASH_CONFIG_MAGIC ||
      cfg->crc32 != crc32(&cfg->entries[0], sizeof(cfg->entries))) {
    return false;
  }

  for (int i = 0; i < MAX_CONFIG_ENTRIES; i++) {
    const flash_config_t *e = &cfg->entries[i];
    if (e->cfg_type > CFG_TYPE_VSS) {
      puts("Invalid config: unknown cfg_type at index ");
      puth(i);
      puts("\n");
      return false;
    }

    if (e->cfg_type == CFG_TYPE_ADC){
      puts("adc config found at index: ");
      puth(i);
      puts("\n");
    }

    if (e->cfg_type == CFG_TYPE_CAN && e->can.sig_type != i) {
      puts("Invalid CAN config: sig_type does not match index ");
      puth(i);
      puts("\n");
      return false;
    }
  }

  return true;
}

void init_config_pointers(const config_block_t *cfg) {
  for (int i = 0; i < MAX_CONFIG_ENTRIES; i++) {
    const flash_config_t *e = &cfg->entries[i];

    // Skip completely disabled configs
    if (e->cfg_type == 0) continue;

    // For CAN configs, make sure sig_type matches index
    if (e->cfg_type == CFG_TYPE_CAN && e->can.sig_type != i) {
      puts("Config sig_type/index mismatch at index ");
      puth(i);
      puts(" (sig_type = ");
      puth(e->can.sig_type);
      puts(")\n");
      continue;
    }

    // Accept and assign all valid config types
    signal_configs[i] = e;

    puts("Configured signal type ");
    puth(e->cfg_type);
    puts("\n");
  }
}

static inline int32_t extract_scaled_signal(const flash_config_t *cfg, CAN_FIFOMailBox_TypeDef *msg) {
  uint64_t can_raw = 0;
  for (int i = 0; i < cfg->can.msg_len_bytes; i++) {
    can_raw = (can_raw << 8) & 0xFFFFFFFFFFFFFFFFULL;
    can_raw |= GET_BYTE(msg, i);
  }

  uint32_t raw_val = (can_raw >> cfg->can.shift_amt) & ((1ULL << cfg->can.sig_len) - 1);

  // Sign-extend if needed
  if (cfg->can.is_signed & 1) {
    // If the sign bit is set
    if (raw_val & (1U << (cfg->can.sig_len - 1))) {
      raw_val |= ~((1U << cfg->can.sig_len) - 1);  // fill upper bits with 1's
    }
  }

  int32_t val = (int32_t)raw_val;
  val = val * cfg->can.scale_mult + cfg->can.scale_offs;
  return val;
}