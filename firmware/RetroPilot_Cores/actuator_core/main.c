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

#include "drivers/uart.h"

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

#include "chimera/usb.h"
#include "chimera/chimera.h"

#define PEDAL_USB

// TODO: SPI and PWM
// TODO: integrate Chimera

// #define CAN CAN1

// #define DEBUG

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

#define USB_CTRL_TIMEOUT 50
uint32_t ctrl_timeout = 0;
bool usb_ctrl_active = false;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}
#include "RetroPilot_Cores/actuator_core/can.h"

void __attribute__ ((noinline)) enable_fpu(void) {
  // enable the FPU
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
}
// ********************* serial debugging *********************

// ********************* usb debugging *********************
void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

// callback function to handle flash writes over USB
void usb_cb_ep0_out(void *usbdata_ep0, int len_ep0) {
  if ((usbdata_ep0 == last_usb_data_ptr) & (len_ep0 == sizeof(flash_config_t))) {
    puts("debug_1\n");
    handle_deferred_config_write();
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
  uint8_t *usbdata8 = (uint8_t *)usbdata;
  uart_ring *ur = get_ring_by_number(usbdata8[0]);
  if ((len != 0) && (ur != NULL)) {
    if ((usbdata8[0] < 2U)) {
      for (int i = 1; i < len; i++) {
        while (!putc(ur, usbdata8[i])) {
          // wait
        }
      }
    }
  }
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
  puts("USB enumeration complete\n");
  is_enumerated = 1;
}

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
      // this allows reflashing of the bootstub
      // so it's blocked over wifi
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
    // **** 0xf1: Clear CAN ring buffer.
    case 0xf1:
      if (setup->b.wValue.w == 0xFFFFU) {
        puts("Clearing CAN Rx queue\n");
        can_clear(&can_rx_q);
      } else if (setup->b.wValue.w < BUS_MAX) {
        puts("Clearing CAN Tx queue\n");
        can_clear(can_queues[setup->b.wValue.w]);
      } else {
        puts("Clearing CAN CAN ring buffer failed: wrong bus number\n");
      }
      break;
    // **** 0xf2: Clear UART ring buffer.
    case 0xf2:
      {
        uart_ring * rb = get_ring_by_number(setup->b.wValue.w);
        if (rb != NULL) {
          puts("Clearing UART queue.\n");
          clear_uart_buff(rb);
        }
        break;
      }
    case 0xFB: {
      uint16_t offset = setup->b.wValue.w;
      uint16_t length = setup->b.wLength.w;
      
      static char json_buffer[256];
      static uint16_t json_len = 0;
      
      if (offset == 0) {
        json_var_t response_vars[] = {
          {"gas_set", gas_set, JSON_FMT_DEC, 0, 0},
          {"enabled", enabled, JSON_FMT_DEC, 0, 0},
          {"adc0", adc[0], JSON_FMT_DEC, 0, 0},
          {"adc1", adc[1], JSON_FMT_DEC, 0, 0},
          {"state", state, JSON_FMT_DEC, 0, 0}
        };
        json_len = json_output("actuator_status", response_vars, 5, json_buffer, sizeof(json_buffer));
      }
      
      uint16_t max_len = MIN(length, MAX_RESP_LEN);
      if (offset < json_len) {
        uint16_t remaining = json_len - offset;
        uint16_t copy_len = MIN(remaining, max_len);
        memcpy(resp, json_buffer + offset, copy_len);
        resp_len = copy_len;
      }
      break;
    }
    case 0xFC: {
      ctrl_timeout = 0;
      usb_ctrl_active = true;
      
      if (setup->b.wLength.w > 0) {
        uint32_t ctrl_val = 0;
        json_key_map_t actuator_key_map[] = {
          {"ctrl_in", &ctrl_val, 0, 2000}
        };
        
        if (json_parse_input((char*)usbdata, setup->b.wLength.w, actuator_key_map, 1) == JSON_PARSE_OK) {
          gas_set = ctrl_val;
          enabled = (gas_set > 0) ? 1 : 0;
        }
      }
      break;
    }
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
// control theory:
// set clutch/relay thing
// get to requested position 
uint8_t enabled = 0;
uint32_t tps_min = 0;
uint32_t tps_max = 0;
uint16_t adc_tol = 0;
uint8_t adc_num = 0;
// addresses to be used on CAN
#define CAN_GAS_INPUT  0x400
#define CAN_GAS_OUTPUT 0x401U
#define CAN_OUTPUT_ACC_STATE 0x150
#define CAN_GAS_SIZE 6
#define COUNTER_CYCLE 0xFU

// position and clutch
uint32_t gas_set = 0;
bool clutch = 0;

#define MAX_TIMEOUT 20U
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

#define FAULT_CONFIG_INVALID 14U
#define FAULT_NOT_CONFIGURED 15U

uint8_t state = FAULT_STARTUP;

const uint8_t crc_poly = 0x1D;  // standard crc8
uint8_t crc8_lut_1d[256];

uint32_t adc[2];

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN1->TSR |= CAN_TSR_RQCP0;
}

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {
    #ifdef DEBUG
      //puts("CAN RX\n");
    #endif
    int address = CAN1->sFIFOMailBox[0].RIR >> 21;
    if (address == CAN_GAS_INPUT) {
      // softloader entry
      if (GET_BYTES_04(&CAN1->sFIFOMailBox[0]) == 0xdeadface) {
        if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x0ab00b1e) {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        } else if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x02b00b1e) {
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        } else {
          puts("Failed entering Softloader or Bootloader\n");
        }
      }

      // normal packet
      uint8_t dat[6];
      for (int i=0; i<6; i++) {
        dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
      }
      uint16_t value_0 = (dat[2] << 8) | dat[1];
      bool enable = ((dat[5] >> 7) & 1U) != 0U;
      uint8_t index = dat[5] & COUNTER_CYCLE;
      if (dat[0] == lut_checksum(dat, 6, crc8_lut_1d)) {
        if (((current_index + 1U) & COUNTER_CYCLE) == index) {
          if (enable) {
            enabled = 1;
            if (value_0 < 2000){ 
              gas_set = value_0;
            } else { 
              state = FAULT_INVALID;
            }
          } else {
            enabled = 0;
            gas_set = 0;
            // clear the fault state if values are 0 and the incoming request is valid
            if (value_0 == 0U) {
              state = NO_FAULT;
            } else {
              state = FAULT_INVALID;
            }
          }
          // clear the timeout
          timeout = 0;
        }
        current_index = index;
      } else {
        // wrong checksum = fault
        puts("BAD CHECKSUM");
        state = FAULT_BAD_CHECKSUM;
      }
    }
    // next
    CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN1);
}

unsigned int pkt_idx = 0;
unsigned int pkt2_idx = 0;

bool send = 0;
void TIM3_IRQ_Handler(void) {

  // check timer for sending the user pedal and clearing the CAN
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    dat[1] = (adc[0] >> 0) & 0xFFU;
    dat[2] = (adc[0] >> 8) & 0xFFU;
    dat[3] = (adc[1] >> 0) & 0xFFU;
    dat[4] = (adc[1] >> 8) & 0xFFU;
    dat[5] = (0 >> 0) & 0xFFU;
    dat[6] = (0 >> 8) & 0xFFU;
    dat[7] = ((state & 0xFU) << 4) | pkt_idx;
    dat[0] = lut_checksum(dat, 8, crc8_lut_1d);
    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
    to_send.RDTR = 8;
    to_send.RIR = (CAN_GAS_OUTPUT << 21) | 1U;
    can_send(&to_send, 0, false);
    llcan_clear_send(CAN1);
    ++pkt_idx;
    pkt_idx &= COUNTER_CYCLE;
    send = !send;
  } else { 
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      // puts("CAN MISS\n");
    #endif
  }

  TIM3->SR = 0;

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
    set_gpio_output(GPIOB, 4, 1);
    set_gpio_output(GPIOA, 2, 0);
    set_gpio_output(GPIOA, 3, 0);
    set_gpio_output(GPIOA, 4, 0);
    set_gpio_output(GPIOA, 5, 0);
  } else {
    if (timeout < MAX_TIMEOUT){
      timeout += 1U;
    }
  }

  if (usb_ctrl_active){
    ctrl_timeout++;
    if (ctrl_timeout > USB_CTRL_TIMEOUT){
      usb_ctrl_active = false;
      ctrl_timeout = 0;
    }
  }
}

void TIM1_BRK_TIM9_IRQ_Handler(void) {
  if (TIM9->SR != 0) {
    // const flash_config_t *cfg = NULL;
    // cfg = signal_configs[0];
    if (usb_ctrl_active){
      json_var_t status_vars[] = {
        {"adc0", adc[0], JSON_FMT_DEC, 0, 0},
        {"adc1", adc[1], JSON_FMT_DEC, 0, 0},
        {"state", state, JSON_FMT_DEC, 0, 0},
        {"timeout", timeout, JSON_FMT_DEC, 0, 0},
      };
      json_output("actuator_status", status_vars, sizeof(status_vars)/sizeof(status_vars[0]), NULL, 0);
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

// ***************************** main code *****************************
void loop(void) {

  adc[0] = adc_get(12);
  adc[1] = adc_get(13);

  if (enabled && gas_set > 0) {
    if (gas_set > (tps_max - tps_min)) gas_set = tps_max - tps_min;

    uint32_t target_adc = gas_set + tps_min;
    uint32_t lower_deadzone = (target_adc > adc_tol) ? target_adc - adc_tol : 0;
    uint32_t upper_deadzone = target_adc + adc_tol;

    if (adc[adc_num] >= lower_deadzone && adc[adc_num] <= upper_deadzone) {
      // In deadzone
      set_gpio_output(GPIOA, 2, 0);  // PWM off
      set_gpio_output(GPIOB, 10, 1); // disable motor
    } else if (adc[adc_num] < target_adc) {
      // Need to increase throttle
      set_gpio_output(GPIOB, 10, 0); // enable
      set_gpio_output(GPIOB, 0, 1);  // direction
      set_gpio_output(GPIOA, 2, 1);  // PWM on
    } else {
      // Need to decrease throttle
      set_gpio_output(GPIOB, 10, 0); // enable
      set_gpio_output(GPIOB, 0, 0);  // reverse direction
      set_gpio_output(GPIOA, 2, 1);  // PWM on
    }

    set_gpio_output(GPIOB, 3, 0);  // engage clutch (Motor2)
    set_gpio_output(GPIOA, 3, 1);
  } else {
    // Disable everything
    set_gpio_output(GPIOB, 3, 1); // disable driver 2 (clutch)
    set_gpio_output(GPIOA, 3, 0);
    // set_gpio_output(GPIOB, 4, 1);  // clutch off
    set_gpio_output(GPIOA, 2, 0);  // PWM off
    set_gpio_output(GPIOB, 10, 0); // disable motor
    set_gpio_output(GPIOB, 0, 1);  // default DIR
    gas_set = 0;
    }

  watchdog_feed();
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
    puts("FLASH NOT FORMATTED. PLEASE FORMAT BEFORE WRITING A CONFIG.\n");
  } else {
    init_config_pointers(&current_cfg_in_ram);
    puts("FLASH VALIDATED\n");
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

  set_gpio_mode(GPIOC, 3, MODE_ANALOG);

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

  // clutch
  set_gpio_mode(GPIOB, 4, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 4, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOB, 4, PULL_DOWN);
  set_gpio_output(GPIOB, 4, 1); // clutch off
  // motors
  set_gpio_mode(GPIOA, 2,  MODE_OUTPUT); // PWM MOTOR1
  set_gpio_mode(GPIOB, 10, MODE_OUTPUT); // DIS MOTOR1
  set_gpio_mode(GPIOB, 0,  MODE_OUTPUT); // DIR MOTOR1

  set_gpio_mode(GPIOA, 3,  MODE_OUTPUT); // PWM MOTOR2
  set_gpio_mode(GPIOB, 3,  MODE_OUTPUT); // DIS MOTOR2
  set_gpio_mode(GPIOB, 1,  MODE_OUTPUT); // DIR MOTOR2

  set_gpio_mode(GPIOA, 10, MODE_OUTPUT); // CSN
  // SCK PA5
  // MOSI PA7
  // MISO PA6

  set_gpio_output_type(GPIOA, 2,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 10, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 0,  OUTPUT_TYPE_PUSH_PULL);

  set_gpio_output_type(GPIOA, 3,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 3,  OUTPUT_TYPE_PUSH_PULL);  
  set_gpio_output_type(GPIOB, 1,  OUTPUT_TYPE_PUSH_PULL);
 
  set_gpio_output_type(GPIOA, 10, OUTPUT_TYPE_PUSH_PULL);
 
  set_gpio_pullup(GPIOA, 2,  PULL_DOWN);
  set_gpio_pullup(GPIOB, 10, PULL_UP);
  set_gpio_pullup(GPIOB, 0,  PULL_DOWN);

  set_gpio_pullup(GPIOA, 3,  PULL_DOWN);
  set_gpio_pullup(GPIOB, 3,  PULL_UP);
  set_gpio_pullup(GPIOB, 1,  PULL_DOWN);

  set_gpio_pullup(GPIOA, 10, PULL_UP); // CSN Active Low

  set_gpio_output(GPIOA, 2,  0);
  set_gpio_output(GPIOB, 10, 1); // Motor1 disabled at boot
  set_gpio_output(GPIOB, 0,  0);

  set_gpio_output(GPIOA, 3,  0);
  set_gpio_output(GPIOB, 3,  1); // Motor2 disabled at boot
  set_gpio_output(GPIOB, 1,  0);

  set_gpio_output(GPIOA, 10, 0); //CSN

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
    puts("WATCHDOG ENABLED\n");
    watchdog_init();
  } else {
    puts("WATCHDOG DISABLED\n");
  }

  if (signal_configs[1] != NULL){
    //&& (signal_configs[1]->cfg_type == CFG_TYPE_ADC) 
    //&& signal_configs[1]->adc.adc_en && signal_configs[1]->adc.adc_num >= 1
    //&& signal_configs[1]->adc.adc1 < signal_configs[1]->adc.adc2 
    tps_min = signal_configs[1]->adc.adc1;
    tps_max = signal_configs[1]->adc.adc2; 
    adc_tol = signal_configs[1]->adc.adc_tolerance;
    adc_num = signal_configs[1]->adc.adc_num;
    puts("ADC configured\n");
    puts("MIN: "); puth(tps_min);
    puts(" MAX: "); puth(tps_max);
    puts(" TOL: "); puth(adc_tol);
    puts(" ADC# "); puth(adc_num); puts("\n");
  } else {
    puts ("ADC not configured. Please use the config tool to set.\n");
    state = FAULT_NOT_CONFIGURED;
  }

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();
  adc[0] = 0;
  adc[1] = 0;
  // main program loop
  while (1) {
    loop();
  }

  return 0;
}
