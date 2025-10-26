// Includes
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

#include "drivers/pwm.h"
// #include "drivers/uart.h"
#include "drivers/json_rpc.h"

#define USB_STRING_DESCRIPTORS

#define STRING_DESCRIPTOR_HEADER(size) \
  (((((size) * 2) + 2) & 0xFF) | 0x0300)

static const uint16_t string_manufacturer_desc[] = {
  STRING_DESCRIPTOR_HEADER(10),
  'R','e','t','r','o','P','i','l','o','t'
};

static const uint16_t string_product_desc[] = {
  STRING_DESCRIPTOR_HEADER(13),
  'A','c','t','u','a','t','o','r',' ','C','o','r','e'
};

// Null functions to replace UART functionality
void puts(const char *a) { UNUSED(a); }
void putch(const char a) { UNUSED(a); }
void puth(unsigned int i) { UNUSED(i); }
void puth2(unsigned int i) { UNUSED(i); }

#include "chimera/usb.h"
#include "chimera/chimera.h"

// USB data buffer (defined in usb.h but need to declare extern)
extern uint8_t usbdata[0x100];

#define DEBUG


uint32_t enter_bootloader_mode;

uint32_t ctrl_timeout = 0;
volatile bool usb_ctrl_active = false;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}
#include "RetroPilot_Cores/actuator_core/can.h"
#include "RetroPilot_Cores/actuator_core/modes/common.h"

// Mode definitions
#define MODE_DEFAULT 0
#define MODE_STEER   1
#define MODE_CRUISE  2

// Mode headers
#include "RetroPilot_Cores/actuator_core/modes/default.h"
#include "RetroPilot_Cores/actuator_core/modes/steer.h"
#include "RetroPilot_Cores/actuator_core/modes/cruise.h"
#include "RetroPilot_Cores/actuator_core/json_rpc_handlers.h"

// Mode function pointers
void (*mode_init_func)(void);
void (*mode_process_func)(void);
void (*mode_can_rx_handler_func)(int address, uint8_t *dat);
void (*mode_timer_handler_func)(void);

// Current mode
uint8_t current_mode = MODE_DEFAULT;

uint8_t enabled = 0;
uint32_t gas_set = 0;
uint8_t state = FAULT_STARTUP;
uint32_t adc[2];

int16_t steer_torque_req = 0;
int16_t steer_angle_req = 0;
uint8_t steer_mode = 0;
uint32_t tps_min = 0;
uint32_t tps_max = 0;
uint16_t adc_tol = 0;
uint8_t adc_num = 0;
uint32_t timeout = 0;
uint32_t current_index = 0;
unsigned int pkt_idx = 0;
const uint8_t crc_poly = 0x1D;
uint8_t crc8_lut_1d[256];

uint8_t motor1_pwm = 0;
uint8_t motor2_pwm = 0;
uint8_t motor1_dir = 1;
uint8_t motor2_dir = 1;
uint8_t motor1_enable = 0;
uint8_t motor2_enable = 0;
uint8_t relay_state = 0;
bool ctrl_enable = false;

extern USB_Setup_TypeDef last_setup_pkt;
extern uint8_t *last_usb_data_ptr;
extern volatile bool config_write_pending;

void __attribute__ ((noinline)) enable_fpu(void) {
  // enable the FPU
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
}

// Serial debugging

// USB debugging
// void debug_ring_callback(uart_ring *ring) {
//   char rcv;
//   while (getc(ring, &rcv) != 0) {
//     (void)putc(ring, rcv);
//   }
// }

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
      json_rpc_handle_request((char*)usbdata_ep0, len_ep0);
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
  UNUSED(hardwired);
  CAN_FIFOMailBox_TypeDef *reply = (CAN_FIFOMailBox_TypeDef *)usbdata;
  int ilen = 0;
  while (ilen < MIN(len/0x10, 4) && can_pop(&can_rx_q, &reply[ilen])) {
    ilen++;
  }
  return ilen*0x10;
}
// send on serial, first byte to select the ring
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  UNUSED(len);
  UNUSED(usbdata);
  // uint8_t *usbdata8 = (uint8_t *)usbdata;
  // uart_ring *ur = get_ring_by_number(usbdata8[0]);
  // if ((len != 0) && (ur != NULL)) {
  //   if ((usbdata8[0] < 2U)) {
  //     for (int i = 1; i < len; i++) {
  //       while (!putc(ur, usbdata8[i])) {
  //         // wait
  //       }
  //     }
  //   }
  // }
}
// send on CAN
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out_complete() {
  if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_BULK_TRANSFER)) {
    usb_outep3_resume_if_paused();
  }
}

void usb_cb_enumeration_complete() {
  // puts("USB enumeration complete\n");
  is_enumerated = 1;
}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  // uart_ring *ur = NULL;

  // Store setup for deferred processing
  memcpy(&last_setup_pkt, setup, sizeof(USB_Setup_TypeDef));
  last_usb_data_ptr = usbdata; 


  switch (setup->b.bRequest) {
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      // this allows reflashing of the bootstub
      // so it's blocked over wifi
      switch (setup->b.wValue.w) {
        case 0:
          // only allow bootloader entry on debug builds
          #ifdef ALLOW_DEBUG
            if (hardwired) {
              // puts("-> entering bootloader\n");
              enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
              NVIC_SystemReset();
            }
          #endif
          break;
        case 1:
          //puts("-> entering softloader\n");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
        default:
          //puts("Bootloader mode invalid\n");
          break;
      }
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    // **** 0xe0: uart read
    // case 0xe0:
    //   ur = get_ring_by_number(setup->b.wValue.w);
    //   if (!ur) {
    //     break;
    //   }
    //   // read
    //   while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
    //                      getc(ur, (char*)&resp[resp_len])) {
    //     ++resp_len;
    //   }
    //   break;
    // **** 0xf1: Clear CAN ring buffer.
    case 0xf1:
      if (setup->b.wValue.w == 0xFFFFU) {
        //puts("Clearing CAN Rx queue\n");
        can_clear(&can_rx_q);
      } else if (setup->b.wValue.w < BUS_MAX) {
        //puts("Clearing CAN Tx queue\n");
        can_clear(can_queues[setup->b.wValue.w]);
      } else {
        //puts("Clearing CAN CAN ring buffer failed: wrong bus number\n");
      }
      break;
    // **** 0xf2: Clear UART ring buffer.
    // case 0xf2:
    //   {
    //     uart_ring * rb = get_ring_by_number(setup->b.wValue.w);
    //     if (rb != NULL) {
    //       //puts("Clearing UART queue.\n");
    //       clear_uart_buff(rb);
    //     }
    //     break;
    //   }
    case 0xFB: // JSON response output
      resp_len = json_rpc_get_response((char*)resp, setup->b.wValue.w, MAX_RESP_LEN);
      if (resp_len <= 0) {
        // puts("JSON RPC get response failed\n");
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
      if (setup->b.wLength.w == sizeof(flash_config_t) &&
          setup->b.wValue.w < MAX_CONFIG_ENTRIES) {
        config_write_pending = true;
      }
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
        } else {
          resp_len = 0;
        }
      } else {
        memset(resp, 0xFF, MIN(length, MAX_RESP_LEN));
        resp_len = MIN(length, MAX_RESP_LEN);
      }
      break;
    }
    default:
      break;
  }
  return resp_len;
}

// Mode management

void setup_mode(uint8_t mode) {
  current_mode = mode;
  //puts("Setting actuator mode: ");
  //puth(mode);
  //puts(" (");
  switch (mode) {
    case MODE_STEER: //puts("STEER"); break;
    case MODE_CRUISE: //puts("CRUISE"); break;
    default: //puts("DEFAULT");
      break;
  }
  //puts(")\n");
  
  switch (mode) {
    case MODE_STEER:
      mode_init_func = steer_init;
      mode_process_func = steer_process;
      mode_can_rx_handler_func = steer_can_rx_handler;
      mode_timer_handler_func = steer_timer_handler;
      break;
    case MODE_CRUISE:
      mode_init_func = cruise_init;
      mode_process_func = cruise_process;
      mode_can_rx_handler_func = cruise_can_rx_handler;
      mode_timer_handler_func = cruise_timer_handler;
      break;
    default:
      mode_init_func = default_init;
      mode_process_func = default_process;
      mode_can_rx_handler_func = default_can_rx_handler;
      mode_timer_handler_func = default_timer_handler;
      break;
  }
  
  if (mode_init_func) {
    mode_init_func();
  }
}

uint8_t detect_mode_from_flash(void) {
  const config_block_t *cfg = (const config_block_t *)CONFIG_FLASH_ADDR;
  
  if (!validate_flash_config(cfg)) {
    return MODE_DEFAULT;
  }
  
  const flash_config_t *sys_cfg = &cfg->entries[0];
  if (sys_cfg->cfg_type == CFG_TYPE_SYS) {
    uint8_t mode = sys_cfg->sys.cfg_extra[0];
    return mode;
  }
  
  return MODE_DEFAULT;
}

// CAN handlers

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN1->TSR |= CAN_TSR_RQCP0;
}

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {
    int address = CAN1->sFIFOMailBox[0].RIR >> 21;
    uint8_t dat[8];
    for (int i = 0; i < 8; i++) {
      dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
    }
    if (mode_can_rx_handler_func) {
      mode_can_rx_handler_func(address, dat);
    }
    CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN1);
}


void TIM3_IRQ_Handler(void) {
  if (mode_timer_handler_func) {
    mode_timer_handler_func();
  }
  
  if (usb_ctrl_active) {
    ctrl_timeout++;
    if (ctrl_timeout > USB_CTRL_TIMEOUT) {
      usb_ctrl_active = false;
      ctrl_timeout = 0;
    }
  }
  
  TIM3->SR = 0;
}

void TIM1_BRK_TIM9_IRQ_Handler(void) {
  if (usb_ctrl_active) {
    ctrl_timeout++;
    if (ctrl_timeout > USB_CTRL_TIMEOUT) {
      usb_ctrl_active = false;
      ctrl_timeout = 0;
    }
  }
  TIM9->SR = 0;
}

void EXTI9_5_IRQ_Handler(void) {
  volatile unsigned int pr = EXTI->PR & (1U << 9);
  if ((pr & (1U << 9)) != 0U) {
    EXTI->PR = (1U << 9);
  }
}

// Main
void loop(void) {
  if (mode_process_func) {
    mode_process_func();
  }
}

int main(void) {
  // ############## FLASH HANDLING #####################
  config_block_t current_cfg_in_ram;
  memcpy(&current_cfg_in_ram, (void *)CONFIG_FLASH_ADDR, sizeof(config_block_t));

  if (!validate_flash_config(&current_cfg_in_ram)) {
    // flash_unlock();
    // flash_config_format();
    // flash_lock();
    // NVIC_SystemReset();
    //puts("FLASH NOT FORMATTED. PLEASE FORMAT BEFORE WRITING A CONFIG.\n");
  } else {
    init_config_pointers(&current_cfg_in_ram);
    //puts("FLASH VALIDATED\n");
  }

  // ###################################################################

  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)

  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  // // 8Hz timer
  REGISTER_INTERRUPT(TIM1_BRK_TIM9_IRQn, TIM1_BRK_TIM9_IRQ_Handler, 10U, FAULT_INTERRUPT_RATE_TIM9)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  enable_fpu();
  
  // No debug UART - PWM uses PA2/PA3
  
  detect_configuration();
  detect_board_type();

  // init board
  current_board->init();
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  usb_init();

  // 8hz
  timer_init(TIM9, 183);
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

  REGISTER_INTERRUPT(EXTI9_5_IRQn, EXTI9_5_IRQ_Handler, 0xFFFF, FAULT_INTERRUPT_RATE_TACH)
  register_set(&(SYSCFG->EXTICR[3]), SYSCFG_EXTICR3_EXTI9_PA, 0x00U);
  register_set_bits(&(EXTI->IMR), (1U << 9));
  register_set_bits(&(EXTI->RTSR), (1U << 9));
  // register_set_bits(&(EXTI->FTSR), (1U << 9));
  // NVIC_EnableIRQ(EXTI9_5_IRQn);

  // pedal stuff
  // dac_init();
  adc_init();

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    //puts("Failed to set llcan speed");
  }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);

  // ============ GPIO SETUP ============
  
  // USART1 for debug (PA9=TX, PA10=RX) - already set in common_init_gpio
  
  // ADC input
  set_gpio_mode(GPIOC, 3, MODE_ANALOG);
  
  // Motor 1
  set_gpio_mode(GPIOA, 2, MODE_OUTPUT);
  set_gpio_mode(GPIOB, 10, MODE_OUTPUT);
  set_gpio_mode(GPIOB, 0, MODE_OUTPUT);
  // set_gpio_output_type(GPIOA, 2, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 10, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 0, OUTPUT_TYPE_PUSH_PULL);
  // set_gpio_pullup(GPIOA, 2, PULL_DOWN);
  set_gpio_pullup(GPIOB, 10, PULL_UP);
  set_gpio_pullup(GPIOB, 0, PULL_DOWN);
  // set_gpio_output(GPIOA, 2, 0);
  set_gpio_output(GPIOB, 10, 0); 
  set_gpio_output(GPIOB, 0, 1);
  
  // Motor 2
  set_gpio_mode(GPIOA, 3, MODE_OUTPUT);
  set_gpio_mode(GPIOB, 3, MODE_OUTPUT);
  set_gpio_mode(GPIOB, 1, MODE_OUTPUT);
  // set_gpio_output_type(GPIOA, 3, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 3, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 1, OUTPUT_TYPE_PUSH_PULL);
  // set_gpio_pullup(GPIOA, 3, PULL_DOWN);
  set_gpio_pullup(GPIOB, 3, PULL_UP);
  set_gpio_pullup(GPIOB, 1, PULL_DOWN);
  // set_gpio_output(GPIOA, 3, 0);
  set_gpio_output(GPIOB, 3, 0);  
  set_gpio_output(GPIOB, 1, 1);
  
  // RELAY (PB4)
  set_gpio_mode(GPIOB, 4, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 4, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOB, 4, PULL_DOWN);
  set_gpio_output(GPIOB, 4, 1); 

  // enable the FPU
  enable_fpu();
  // setup microsecond timer
  // realoads every 4ish seconds
  TIM2->PSC = 479; //48-1;
  TIM2->ARR = 99999; 
  TIM2->DIER |= TIM_DIER_UIE;
  TIM2->CR1 = TIM_CR1_CEN;
  TIM2->EGR = TIM_EGR_UG;
  // use TIM2->CNT to read
  // reset with TIM2->CNT = 0;

  // NVIC_EnableIRQ(TIM2_IRQn);

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);

  if (signal_configs[0] != NULL && (signal_configs[0]->sys.iwdg_en & 1)){
    //puts("WATCHDOG ENABLED\n");
    watchdog_init();
  } else {
    //puts("WATCHDOG DISABLED\n");
  }

  if (signal_configs[1] != NULL){
    tps_min = signal_configs[1]->adc.adc1;
    tps_max = signal_configs[1]->adc.adc2; 
    adc_tol = signal_configs[1]->adc.adc_tolerance;
    adc_num = signal_configs[1]->adc.adc_num;
    //puts("ADC configured\n");
    //puts("MIN: "); puth(tps_min);
    //puts(" MAX: "); puth(tps_max);
    //puts(" TOL: "); puth(adc_tol);
    //puts(" ADC# "); puth(adc_num); //puts("\n");
  } else {
    puts ("ADC not configured. Please use the config tool to set.\n");
    state = FAULT_NOT_CONFIGURED;
  }

  // Initialize JSON RPC
  if (!json_rpc_init(actuator_methods)) {
    //puts("JSON RPC init failed\n");
  } else {
    //puts("JSON RPC: actuator core ready\n");
  }

  // Setup mode system
  uint8_t mode = detect_mode_from_flash();
  setup_mode(mode);
  
  // pwm on PA2, PA3
  set_gpio_mode(GPIOA, 2, MODE_ALTERNATE);
  set_gpio_mode(GPIOA, 3, MODE_ALTERNATE);
  set_gpio_alternate(GPIOA, 2, GPIO_AF3_TIM9);
  set_gpio_alternate(GPIOA, 3, GPIO_AF3_TIM9);
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
  pwm_init(TIM9, 1);
  pwm_init(TIM9, 2);


  //puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();
  adc[0] = 0;
  adc[1] = 0;
  // main program loop
  while (1) {
    loop();
  }

  return 0;
}
