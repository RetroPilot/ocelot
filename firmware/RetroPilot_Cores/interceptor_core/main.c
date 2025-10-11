// ********************* Includes *********************
#include "../../config.h"
#include "libc.h"
#include <string.h>

#include "main_declarations.h"
#include "critical.h"
#include "faults.h"

#include "drivers/registers.h"
#include "drivers/interrupts.h"
#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"

#include "board.h"

#include "drivers/clock.h"
#include "drivers/dac.h"
#include "drivers/timer.h"

#include "gpio.h"
#include "crc.h"

#define CAN CAN1

#define DEBUG

#define USB_STRING_DESCRIPTORS

#define STRING_DESCRIPTOR_HEADER(size) \
  (((((size) * 2) + 2) & 0xFF) | 0x0300)

static const uint16_t string_manufacturer_desc[] = {
  STRING_DESCRIPTOR_HEADER(10),
  'R','e','t','r','o','P','i','l','o','t'
};

static const uint16_t string_product_desc[] = {
  STRING_DESCRIPTOR_HEADER(16),
  'I','n','t','e','r','c','e','p','t','o','r',' ','C','o','r','e'
};

#include "drivers/uart.h"
#include "chimera/usb.h"
#include "chimera/chimera.h"
#include "RetroPilot_Cores/interceptor_core/torque_lut.h"

// USB data buffer (defined in usb.h but need to declare extern)
extern uint8_t usbdata[0x100];

// Debug UART ring buffer (defined in uart.h)
extern uart_ring uart_ring_debug;

// Mode definitions
#define MODE_UNCONFIGURED 0
#define MODE_DIFFERENTIAL 1
#define MODE_GAS_PEDAL    2

// Mode headers
#include "RetroPilot_Cores/interceptor_core/modes/default.h"
#include "RetroPilot_Cores/interceptor_core/modes/differential.h"
#include "RetroPilot_Cores/interceptor_core/modes/gas_pedal.h"// Mode headers
#include "RetroPilot_Cores/interceptor_core/modes/default.h"
#include "RetroPilot_Cores/interceptor_core/modes/differential.h"
#include "RetroPilot_Cores/interceptor_core/modes/gas_pedal.h"

// Mode function pointers
void (*mode_init_func)(void);
void (*mode_process_func)(void);
void (*mode_can_rx_handler_func)(int address, uint8_t *dat);
void (*mode_timer_handler_func)(void);
bool (*mode_validate_adc_func)(uint32_t adc_val, uint8_t channel);
void (*mode_debug_output_func)(bool relay_state);

// Current mode
uint8_t current_mode = MODE_UNCONFIGURED;

uint32_t enter_bootloader_mode;

// Missing constants
#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
#define ENTER_SOFTLOADER_MAGIC 0xdeadc0de

// Function declaration for missing USB callback
void usb_cb_ep0_out(void *usbdata, int len) {
  // Handle flash config writes
  if ((usbdata == last_usb_data_ptr) & (len == sizeof(flash_config_t))) {
    handle_deferred_config_write();
  }
}

// External declarations for chimera.h static variables
extern USB_Setup_TypeDef last_setup_pkt;
extern uint8_t *last_usb_data_ptr;
extern volatile bool config_write_pending;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
  return 0;
}
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out_complete(void) {}
void usb_cb_enumeration_complete(void) {}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;

  // Store the setup packet and data pointer for deferred processing
  memcpy(&last_setup_pkt, setup, sizeof(USB_Setup_TypeDef));
  last_usb_data_ptr = usbdata;  // Use global USB data buffer like chimera

  switch (setup->b.bRequest) {
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      switch (setup->b.wValue.w) {
        case 0:
          // only allow bootloader entry on debug builds
          #ifdef ALLOW_DEBUG
            if (hardwired) {
              puts("-> entering bootloader\n");
              enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
              NVIC_SystemReset();
            }
          #endif
          break;
        case 1:
          puts("-> entering softloader\n");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
        default:
          puts("Bootloader mode invalid\n");
          break;
      }
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    // **** 0xdd: get interceptor mode
    case 0xdd:
      resp[0] = current_mode;
      resp_len = 1;
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);      
      // read
      if (!ur) {
        break;
      }
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    // **** 0xFD: flash config format
    case 0xFD: {
      flash_unlock();
      flash_config_format();
      flash_lock();
      break;
    }
    case 0xFF: {  // Chunked config read
      uint16_t offset = setup->b.wValue.w;
      uint16_t length = setup->b.wLength.w;

      // Make sure we're not reading past the end of the config block
      const uint8_t *flash_bytes = (const uint8_t *)CONFIG_FLASH_ADDR;
      if (*(uint32_t *)flash_bytes == FLASH_CONFIG_MAGIC) {
        uint16_t max_len = MIN(length, MAX_RESP_LEN);  // cap to USB response size
        if (offset < sizeof(config_block_t)) {
          uint16_t remaining = sizeof(config_block_t) - offset;
          uint16_t copy_len = MIN(remaining, max_len);
          memcpy(resp, flash_bytes + offset, copy_len);
          resp_len = copy_len;
          // puts("Chunked config block read\n");
        } else {
          puts("Offset out of range in config read\n");
        }
      } else {
        puts("No valid config block in flash\n");
        memset(resp, 0xFF, MIN(length, MAX_RESP_LEN));
        resp_len = MIN(length, MAX_RESP_LEN);
      }
      break;
    }
    case 0xFE: {  // Write config entry
      puts("0xFE command received. Deferring write...\n");
      if (setup->b.wLength.w == sizeof(flash_config_t) &&
          setup->b.wValue.w < MAX_CONFIG_ENTRIES) {
        config_write_pending = true;
      } else {
        puts("\n len: ");
        puth(setup->b.wLength.w);
        puts("\n expected: ");
        puth(sizeof(flash_config_t));
        puts("\n value: ");
        puth(setup->b.wValue.w);
        puts("0xFE command: Invalid length or index. Not deferring.\n");
      }
      break;
    }
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

// ***************************** can port ***************************
// addresses to be used on CAN

#define CAN_GAS_INPUT  0x300
#define CAN_GAS_OUTPUT 0x301U
#define CAN_GAS_SIZE 6
#define COUNTER_CYCLE 0xFU

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

// two independent values
uint16_t gas_set_0 = 0;
uint16_t gas_set_1 = 0;
int16_t req_mag = 0;;

bool enable = 0;
uint16_t vehicle_speed = 0;

#define MAX_TIMEOUT 50U
uint32_t timeout = 0;
uint32_t timeout_vss = 0;
uint32_t current_index = 0;
uint32_t current_index_vss = 0;

// Fault states are now defined in common.h

uint8_t state = FAULT_STARTUP;

const uint8_t crc_poly = 0x1D;  // standard crc8
uint8_t crc8_lut_1d[256];

void setup_mode(uint8_t mode) {
  current_mode = mode;
  puts("Setting interceptor mode: ");
  puth(mode);
  puts(" (");
  switch (mode) {
    case MODE_DIFFERENTIAL: puts("DIFFERENTIAL"); break;
    case MODE_GAS_PEDAL: puts("GAS_PEDAL"); break;
    default: puts("UNCONFIGURED"); break;
  }
  puts(")\n");
  
  switch (mode) {
    case MODE_DIFFERENTIAL:
      mode_init_func = differential_init;
      mode_process_func = differential_process;
      mode_can_rx_handler_func = differential_can_rx_handler;
      mode_timer_handler_func = differential_timer_handler;
      mode_validate_adc_func = differential_validate_adc;
      mode_debug_output_func = differential_debug_output;
      break;
    case MODE_GAS_PEDAL:
      mode_init_func = gas_pedal_init;
      mode_process_func = gas_pedal_process;
      mode_can_rx_handler_func = gas_pedal_can_rx_handler;
      mode_timer_handler_func = gas_pedal_timer_handler;
      mode_validate_adc_func = gas_pedal_validate_adc;
      mode_debug_output_func = gas_pedal_debug_output;
      break;
    default:
      mode_init_func = default_init;
      mode_process_func = default_process;
      mode_can_rx_handler_func = default_can_rx_handler;
      mode_timer_handler_func = default_timer_handler;
      mode_validate_adc_func = default_validate_adc;
      mode_debug_output_func = default_debug_output;
      break;
  }
}

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) {
    int address = CAN->sFIFOMailBox[0].RIR >> 21;
    uint8_t dat[8];
    for (int i=0; i<8; i++) {
      dat[i] = GET_BYTE(&CAN->sFIFOMailBox[0], i);
    }
    mode_can_rx_handler_func(address, dat);
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN);
}

// Universal ADC/DAC variables for all modes
uint32_t adc_input_0 = 0;  // Primary analog input (e.g., gas pedal position)
uint32_t adc_input_1 = 0;  // Secondary analog input (e.g., brake pedal position)
uint32_t dac_output_0 = 0; // Primary analog output
uint32_t dac_output_1 = 0; // Secondary analog output

// Legacy aliases for compatibility
#define pdl0 adc_input_0
#define pdl1 adc_input_1

bool override = 0;
int32_t magnitude = 0;

unsigned int pkt_idx = 0;

int led_value = 0;

// DAC Safety variables
uint32_t last_dac_0 = DAC_SAFE_CENTER;
uint32_t last_dac_1 = DAC_SAFE_CENTER;

// DAC Safety functions
uint32_t safe_dac_output(uint32_t new_val, uint32_t *last_val, uint8_t channel) {
  UNUSED(channel); // Channel parameter for future use
  
  // Clamp to safe range
  if (new_val < DAC_MIN_SAFE) new_val = DAC_MIN_SAFE;
  if (new_val > DAC_MAX_SAFE) new_val = DAC_MAX_SAFE;
  
  // Rate limiting
  int32_t delta = (int32_t)new_val - (int32_t)*last_val;
  if (delta > MAX_DAC_RATE) {
    new_val = *last_val + MAX_DAC_RATE;
  } else if (delta < -MAX_DAC_RATE) {
    new_val = *last_val - MAX_DAC_RATE;
  }
  
  *last_val = new_val;
  return new_val;
}

bool validate_dac_peripheral(void) {
  // Check if DAC peripheral is responding
  // Read back DAC register values to detect stuck peripheral
  uint32_t dac0_reg = DAC->DHR12R1;
  uint32_t dac1_reg = DAC->DHR12R2;
  
  // Simple sanity check - values should be in valid range
  if (dac0_reg > 4095 || dac1_reg > 4095) {
    return false;
  }
  
  // Check if DAC is enabled
  if (!(DAC->CR & DAC_CR_EN1) || !(DAC->CR & DAC_CR_EN2)) {
    return false;
  }
  
  return true;
}

// 8Hz Safety Timer Handler
void TIM1_BRK_TIM9_IRQ_Handler(void) {
  // ADC validation check
  if (!mode_validate_adc_func(adc_input_0, 0) || !mode_validate_adc_func(adc_input_1, 1)) {
    // Check if it's unconfigured vs sensor fault
    const flash_config_t *cfg0 = signal_configs[1];
    const flash_config_t *cfg1 = signal_configs[2];
    bool adc0_configured = (cfg0 && cfg0->cfg_type == CFG_TYPE_ADC && (cfg0->adc.adc_en & 1));
    bool adc1_configured = (cfg1 && cfg1->cfg_type == CFG_TYPE_ADC && (cfg1->adc.adc_en & 1));
    
    if (!adc0_configured || !adc1_configured) {
      state = FAULT_ADC_UNCONFIGURED;
    } else {
      state = FAULT_SENSOR;
    }
  }
  
  // DAC peripheral validation
  if (!validate_dac_peripheral()) {
    state = FAULT_SENSOR; // Treat DAC peripheral fault as sensor fault
  }

  // Relay turns ON when state is NO_FAULT and enable is 1 for all modes
  // In default mode, relay is always OFF
  bool relay_on = (current_mode != MODE_UNCONFIGURED) && (state == NO_FAULT) && enable;
  set_gpio_output(GPIOB, 0, !relay_on);  // Invert for active-low relay

  #ifdef DEBUG
    mode_debug_output_func(relay_on);
  #endif

  TIM9->SR = 0;
}

void TIM3_IRQ_Handler(void) {
  mode_timer_handler_func();
  TIM3->SR = 0;
}

// ***************************** main code *****************************

uint8_t detect_mode_from_flash(void) {
  const config_block_t *cfg = (const config_block_t *)CONFIG_FLASH_ADDR;
  
  puts("Reading flash config...\n");
  
  // Check if flash config is valid
  if (!validate_flash_config(cfg)) {
    puts("Flash config invalid, using default mode\n");
    return MODE_UNCONFIGURED;
  }
  
  puts("Flash config valid\n");
  
  // Look for SYS config at index 0
  const flash_config_t *sys_cfg = &cfg->entries[0];
  if (sys_cfg->cfg_type == CFG_TYPE_SYS) {
    uint8_t mode = sys_cfg->sys.cfg_extra[0];
    puts("Found SYS config, mode: ");
    puth(mode);
    puts(" (");
    switch (mode) {
      case MODE_DIFFERENTIAL: puts("DIFFERENTIAL"); break;
      case MODE_GAS_PEDAL: puts("GAS_PEDAL"); break;
      default: puts("UNCONFIGURED"); break;
    }
    puts(")\n");
    return mode;
  }
  
  puts("No SYS config found, using default mode\n");
  return MODE_UNCONFIGURED;
}


int main(void) {
  // ######################## FLASH HANDLING ###########################
  config_block_t current_cfg_in_ram;
  memcpy(&current_cfg_in_ram, (void *)CONFIG_FLASH_ADDR, sizeof(config_block_t));

  if (!validate_flash_config(&current_cfg_in_ram)) {
    puts("FLASH NOT FORMATTED. PLEASE FORMAT BEFORE WRITING A CONFIG.\n");
  } else {
    init_config_pointers(&current_cfg_in_ram);
  }
  // ###################################################################


  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)

  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  // 8Hz Safety timer
  REGISTER_INTERRUPT(TIM1_BRK_TIM9_IRQn, TIM1_BRK_TIM9_IRQ_Handler, 10U, FAULT_INTERRUPT_RATE_TIM9)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  // init board
  current_board->init();

  set_gpio_mode(GPIOC, 3, MODE_ANALOG);
  
  // Initialize debug UART
  uart_init(&uart_ring_debug, 115200);

  // enable USB
  usb_init();

  // pedal stuff
  dac_init();
  // Set DAC to safe center values at startup
  dac_set(0, DAC_SAFE_CENTER);
  dac_set(1, DAC_SAFE_CENTER);
  adc_init();

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan speed");
  }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);

  // 8Hz Safety timer
  timer_init(TIM9, 183);
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

  // relay
  set_gpio_mode(GPIOB, 0, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 0, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOB, 0, 1);  // Start with relay OFF (active-low)

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);

  // Setup mode system after hardware init
  uint8_t mode = detect_mode_from_flash();
  setup_mode(mode);

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // watchdog_init();
  
  // main loop
  while (1) {
    mode_process_func();
  }

  return 0;
}
