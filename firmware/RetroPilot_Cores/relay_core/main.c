// ********************* Includes *********************
#include "../../config.h"
#include "libc.h"

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
  STRING_DESCRIPTOR_HEADER(10),
  'R','e','l','a','y',' ','C','o','r','e'
};

#include "drivers/uart.h"
#include "chimera/usb.h"
#include "chimera/chimera.h"
#include "drivers/json_rpc.h"

// External declarations for chimera.h static variables
extern USB_Setup_TypeDef last_setup_pkt;
extern uint8_t *last_usb_data_ptr;
extern volatile bool config_write_pending;

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
#define ENTER_SOFTLOADER_MAGIC 0xdeadc0de
uint32_t enter_bootloader_mode;

#define USB_CTRL_TIMEOUT 50
uint32_t ctrl_timeout = 0;
uint32_t ctrl_in = 0;
volatile bool usb_ctrl_active = false;



// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}
void __attribute__ ((noinline)) enable_fpu(void) {
  // enable the FPU
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
}

uint8_t gpio_state= 0;
uint8_t relay_ctrl = 0;
uint8_t relay_state = 0;  // Persistent relay state for JSON responses

void set_relays(uint8_t relays){
  relay_state = relays;  // Store current state
  uint8_t relay_buf = ~relays;
  set_gpio_output(GPIOA,  4, (relay_buf >> 0U) & 1U); 
  set_gpio_output(GPIOA,  5, (relay_buf >> 1U) & 1U);
  set_gpio_output(GPIOA,  6, (relay_buf >> 2U) & 1U);
  set_gpio_output(GPIOA,  7, (relay_buf >> 3U) & 1U);
  set_gpio_output(GPIOB,  0, (relay_buf >> 4U) & 1U);
  set_gpio_output(GPIOB,  1, (relay_buf >> 5U) & 1U);
  set_gpio_output(GPIOB, 10, (relay_buf >> 6U) & 1U);
  set_gpio_output(GPIOB, 12, (relay_buf >> 7U) & 1U);
}

uint8_t get_inputs(void){
  uint8_t input_buf = 0;
  input_buf |= get_gpio_input(GPIOC, 2) << 0U;
  input_buf |= get_gpio_input(GPIOC, 3) << 1U;
  input_buf |= get_gpio_input(GPIOC, 4) << 2U;
  input_buf |= get_gpio_input(GPIOC, 5) << 3U;
  input_buf |= get_gpio_input(GPIOC, 6) << 4U;
  input_buf |= get_gpio_input(GPIOC, 7) << 5U;
  input_buf |= get_gpio_input(GPIOC, 8) << 6U;
  input_buf |= get_gpio_input(GPIOC, 9) << 7U;
  return ~input_buf;
}

// ********************* serial debugging *********************

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

// callback function to handle flash writes over USB
void usb_cb_ep0_out(void *usbdata_ep0, int len_ep0) {
  // Validate input parameters
  if (usbdata_ep0 == NULL || len_ep0 <= 0 || len_ep0 >= 512) {
    return;
  }
  
  // Validate data content for JSON RPC (must start with '{' for JSON)
  uint8_t *data = (uint8_t*)usbdata_ep0;
  if (len_ep0 > 0 && len_ep0 < 512 && data[0] == '{') {
    if (len_ep0 < 511) {
      ((char*)usbdata_ep0)[len_ep0] = '\0';
      if (!json_rpc_handle_request((char*)usbdata_ep0, len_ep0)) {
        puts("JSON RPC handle request failed\n");
      }
    }
  }
  
  // Handle flash config writes with strict validation
  if ((usbdata_ep0 == last_usb_data_ptr) && (len_ep0 == sizeof(flash_config_t))) {
    if (config_write_pending) {
      handle_deferred_config_write();
    }
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

  // Store the setup packet and data pointer for deferred processing if needed
  // This must be done for all commands that might involve an OUT data phase
  // and need deferred processing.
  memcpy(&last_setup_pkt, setup, sizeof(USB_Setup_TypeDef));
  last_usb_data_ptr = usbdata; // Save the pointer to the buffer

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
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      // read
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    case 0xFB: // JSON response output
      resp_len = json_rpc_get_response((char*)resp, setup->b.wValue.w, MAX_RESP_LEN);
      if (resp_len <= 0) {
        puts("JSON RPC get response failed\n");
        resp_len = 0;
      }
      break;
    case 0xFC: // JSON command input
      // Data will be handled in usb_cb_ep0_out callback
      break;
    case 0xFD: {
      flash_unlock();
      flash_config_format();
      flash_lock();
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
    case 0xFF: {  // Chunked config read
      uint16_t offset = setup->b.wValue.w;
      uint16_t length = setup->b.wLength.w;
      puts("USB Flash Read requested 0xFF/n");

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
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

// ***************************** can port *****************************

// addresses to be used on CAN
#define CAN_OUTPUT 0x601U
#define CAN_INPUT  0x600U
#define CAN_SIZE 5
#define COUNTER_CYCLE 0xFU

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

#define MAX_TIMEOUT 100U
uint32_t timeout = 0;
uint32_t current_index = 0;

// Faults 
// Hardware
#define NO_FAULT 0U
#define FAULT_STARTUP 1U
#define FAULT_SENSOR 2U
// CAN
#define FAULT_SEND 3U
#define FAULT_SCE 4U
#define FAULT_TIMEOUT 5U
// Bad messages
#define FAULT_BAD_CHECKSUM 6U
#define FAULT_INVALID 7U

uint8_t state = FAULT_STARTUP;

const uint8_t crc_poly = 0x1D;  // standard crc8
uint8_t crc8_lut_1d[256];

uint32_t relay_ctrl_can = 0;

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) {
    // #ifdef DEBUG
    //   puts("CAN RX\n");
    // #endif
    int address = CAN->sFIFOMailBox[0].RIR >> 21;
    if (address == CAN_INPUT) {
      // softloader entry
      if (GET_BYTES_04(&CAN->sFIFOMailBox[0]) == 0xdeadface) {
        if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x0ab00b1e) {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        } else if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x02b00b1e) {
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        } else {
          puts("Failed entering Softloader or Bootloader\n");
        }
      }

      // normal packet
      uint8_t dat[CAN_SIZE];
      for (int i=0; i<CAN_SIZE; i++) {
        dat[i] = GET_BYTE(&CAN->sFIFOMailBox[0], i);
      }
      uint32_t value_0 = dat[1] | dat[2] << 8 | dat[3] << 16;
      bool enable = ((dat[4] >> 7) & 1U) != 0U;
      uint8_t index = dat[4] & COUNTER_CYCLE;
      if (dat[0] == lut_checksum(dat, CAN_SIZE, crc8_lut_1d)) {
        if (((current_index + 1U) & COUNTER_CYCLE) == index) {
          timeout = 0;
          state = NO_FAULT;
          if (enable) {
            relay_ctrl_can = value_0;
          } else {
            // clear the fault state if values are 0 and the incoming request is valid
            if (value_0 == 0U) {
              state = NO_FAULT;
            } else {
              state = FAULT_INVALID;
            }
            relay_ctrl_can = 0;
          }
        }
        current_index = index;
      } else {
        // wrong checksum = fault
        state = FAULT_BAD_CHECKSUM;
      }
    }
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN);
}

unsigned int pkt_idx = 0;

uint16_t tick = 0;

// Helper function to build bit array JSON
static int json_build_bit_array(char* buffer, int max_len, uint8_t value) {
  if (!buffer || max_len <= 0) return 0;
  
  char* p = buffer;
  int remaining = max_len - (p - buffer);
  if (remaining <= 0) return 0;
  
  p += json_strcpy_safe(p, "[", remaining);
  for (int i = 0; i < 8; i++) {
    remaining = max_len - (p - buffer);
    if (remaining <= 0) break;
    
    if (i > 0) p += json_strcpy_safe(p, ",", remaining);
    remaining = max_len - (p - buffer);
    if (remaining <= 0) break;
    
    p += json_strcpy_safe(p, ((value >> i) & 1) ? "1" : "0", remaining);
  }
  remaining = max_len - (p - buffer);
  if (remaining > 0) {
    p += json_strcpy_safe(p, "]", remaining);
  }
  return p - buffer;
}

// Simplified JSON handlers
static int json_system_info(const char* params, char* response, int max_len) {
  (void)params;
  if (!response || max_len <= 0) {
    return 0;
  }
  
  char result[256];
  json_build_system_info(result, sizeof(result), "relay_core", "\"gpio\",\"relay\"");
  return json_build_success(response, max_len, result);
}

static int json_relay_set(const char* params, char* response, int max_len) {
  if (!params || !response || max_len <= 0) {
    return json_build_error(response, max_len, "invalid_params", "Invalid parameters");
  }
  
  int value = json_parse_int(params, "value");
  if (value >= 0 && value <= 255) {
    usb_ctrl_active = true;
    ctrl_timeout = 0;
    set_relays(value);
    char result[64];
    char* p = result;
    p += json_strcpy_safe(p, "{\"success\":true,\"relay_ctrl\":", sizeof(result) - (p - result));
    p += json_build_bit_array(p, sizeof(result) - (p - result), relay_state);
    p += json_strcpy_safe(p, "}", sizeof(result) - (p - result));
    return json_build_success(response, max_len, result);
  }
  return json_build_error(response, max_len, "invalid_params", "Value must be 0-255");
}

static int json_relay_get(const char* params, char* response, int max_len) {
  (void)params;
  if (!response || max_len <= 0) {
    return 0;
  }
  
  char result[48];
  char* p = result;
  p += json_strcpy_safe(p, "{\"relays\":", sizeof(result) - (p - result));
  p += json_build_bit_array(p, sizeof(result) - (p - result), relay_state);
  p += json_strcpy_safe(p, "}", sizeof(result) - (p - result));
  return json_build_success(response, max_len, result);
}

static int json_gpio_read(const char* params, char* response, int max_len) {
  (void)params;
  if (!response || max_len <= 0) {
    return 0;
  }
  
  char result[48];
  char* p = result;
  p += json_strcpy_safe(p, "{\"gpio\":", sizeof(result) - (p - result));
  p += json_build_bit_array(p, sizeof(result) - (p - result), gpio_state);
  p += json_strcpy_safe(p, "}", sizeof(result) - (p - result));
  return json_build_success(response, max_len, result);
}

// Simple inline handlers for system methods
static int json_system_methods(const char* params, char* response, int max_len) {
  (void)params;
  if (!response || max_len <= 0) {
    return 0;
  }
  
  return json_build_success(response, max_len,
    "[\"system.info\",\"system.methods\",\"system.reset\",\"relay.set\",\"relay.get\",\"gpio.read\"]");
}

static int json_system_reset(const char* params, char* response, int max_len) {
  (void)params; (void)response; (void)max_len;
  NVIC_SystemReset();
  return 0;
}

// Method registry
static const json_method_t device_methods[] = {
  {"system.info", json_system_info},
  {"system.methods", json_system_methods},
  {"system.reset", json_system_reset},
  {"relay.set", json_relay_set},
  {"relay.get", json_relay_get},
  {"gpio.read", json_gpio_read},
  {NULL, NULL}
};

void TIM1_BRK_TIM9_IRQ_Handler(void) {
  gpio_state = get_inputs();

  if (state != NO_FAULT) {
    relay_ctrl_can = 0;
  }

  if (!usb_ctrl_active) {
    // check relay configs and set relays
    for (int i = 1; i < 9; i++) {
      const flash_config_t *cfg = signal_configs[i];
      if (!cfg || cfg->cfg_type != GFG_TYPE_RELAY) continue;

      // GPIO
      if (cfg->relay.gpio_en & 1U) {
        if ((gpio_state & cfg->relay.gpio_in) == cfg->relay.gpio_in){
          relay_ctrl |= (1U << (i-1));
        }
      }

      // CAN_CTRL_RP
      if (cfg->relay.type & 1U){
        if (relay_ctrl_can == cfg->relay.type){
          relay_ctrl |= (1U << (i-1));
        }
      }
    }
  }
  
  set_relays(relay_ctrl);

  puts("tick: ");
  puth(tick);
  puts("\n");
  puts("relays: ");
  putb(relay_ctrl);
  puts("\n");

  tick++;
  tick &= 0xFFFF;

  relay_ctrl = 0;

  TIM9->SR = 0;
}

void TIM3_IRQ_Handler(void) {

  // check timer for sending the IO state and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[4];
    dat[1] = (~gpio_state >> 0) & 0xFFU;
    dat[2] = (~gpio_state >> 8) & 0xFFU;
    dat[3] = ((state & 0xFU) << 4) | pkt_idx;
    dat[0] = lut_checksum(dat, 4, crc8_lut_1d);
    CAN->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN->sTxMailBox[0].TDHR = 0;
    CAN->sTxMailBox[0].TDTR = 4;  // len of packet is 5
    CAN->sTxMailBox[0].TIR = (CAN_OUTPUT << 21) | 1U;
    ++pkt_idx;
    pkt_idx &= COUNTER_CYCLE;
  } else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      // puts("CAN MISS\n");
    #endif
  }

  // blink the LED
  TIM3->SR = 0;

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }
  timeout &= 0xFFU; // keep it in bounds

  if (usb_ctrl_active) {
    ctrl_timeout++;
    if (ctrl_timeout > USB_CTRL_TIMEOUT) {
      usb_ctrl_active = false;
      ctrl_timeout = 0;
    }
  }
}

// ***************************** main code *****************************
void loop(void) {
  watchdog_feed();
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

  // 8Hz timer
  REGISTER_INTERRUPT(TIM1_BRK_TIM9_IRQn, TIM1_BRK_TIM9_IRQ_Handler, 10U, FAULT_INTERRUPT_RATE_TIM9)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  // init board
  current_board->init();

  // enable USB
  usb_init();

  // pedal stuff
  // dac_init();
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

  // 8Hz timer
  timer_init(TIM9, 183);
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

  // setup GPIO inputs
  set_gpio_mode(GPIOC, 2, MODE_INPUT);
  set_gpio_mode(GPIOC, 3, MODE_INPUT);
  set_gpio_mode(GPIOC, 4, MODE_INPUT);
  set_gpio_mode(GPIOC, 5, MODE_INPUT);
  set_gpio_mode(GPIOC, 6, MODE_INPUT);
  set_gpio_mode(GPIOC, 7, MODE_INPUT);
  set_gpio_mode(GPIOC, 8, MODE_INPUT);
  set_gpio_mode(GPIOC, 9, MODE_INPUT);

  // setup GPIO outputs
  set_gpio_mode(GPIOA, 4,  MODE_OUTPUT);
  set_gpio_mode(GPIOA, 5,  MODE_OUTPUT);
  set_gpio_mode(GPIOA, 6,  MODE_OUTPUT);
  set_gpio_mode(GPIOA, 7,  MODE_OUTPUT);
  set_gpio_mode(GPIOB, 0,  MODE_OUTPUT);
  set_gpio_mode(GPIOB, 1,  MODE_OUTPUT);
  set_gpio_mode(GPIOB, 10, MODE_OUTPUT);
  set_gpio_mode(GPIOB, 12, MODE_OUTPUT);
  set_gpio_pullup(GPIOA, 4,  PULL_UP);
  set_gpio_pullup(GPIOA, 5,  PULL_UP);
  set_gpio_pullup(GPIOA, 6,  PULL_UP);
  set_gpio_pullup(GPIOA, 7,  PULL_UP);
  set_gpio_pullup(GPIOB, 0,  PULL_UP);
  set_gpio_pullup(GPIOB, 1,  PULL_UP);
  set_gpio_pullup(GPIOB, 10, PULL_UP);
  set_gpio_pullup(GPIOB, 12, PULL_UP);
  set_gpio_output_type(GPIOA, 4,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOA, 5,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOA, 6,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOA, 7,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 0,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 1,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 10, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 12, OUTPUT_TYPE_PUSH_PULL);

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);

  // Initialize JSON RPC with custom handler
  if (!json_rpc_init(device_methods)) {
    puts("JSON RPC init failed\n");
  } else {
    puts("JSON RPC: relay_core ready\n");
  }

  if (signal_configs[0] != NULL && (signal_configs[0]->sys.iwdg_en & 1)){
    puts("WATCHDOG ENABLED\n");
    watchdog_init();
  } else {
    puts("WATCHDOG DISABLED\n");
  }

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    loop();
  }

  return 0;
}
