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

// #ifdef PEDAL_USB
//   #include "drivers/uart.h"
//   #include "drivers/usb.h"
// #else

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

#include "chimera/usb.h"
#include "chimera/chimera.h"

  // no serial either
  void puts(const char *a) {
    UNUSED(a);
  }
  void puth(unsigned int i) {
    UNUSED(i);
  }
  void puth2(unsigned int i) {
    UNUSED(i);
  }

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}
void __attribute__ ((noinline)) enable_fpu(void) {
  // enable the FPU
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
}
// ********************* serial debugging *********************

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
  switch (setup->b.bRequest) {
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
#define CAN_SIZE 4
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

uint8_t gpio_lights = 0;
uint8_t can_lights = 0;

#define L_TURN     1U << 15U
#define R_TURN     1U << 14U
#define TAIL       1U << 13U
#define HEADLIGHTS 1U << 12U
#define HIBEAM     (1U << 11U) | HEADLIGHTS

void set_relays(uint8_t relay_buf){
  set_gpio_output(GPIOA,  4, (relay_buf >> 0U) & 1U);  // LTURN
  set_gpio_output(GPIOA,  5, (relay_buf >> 1U) & 1U);  // RTURN
  set_gpio_output(GPIOA,  6, (relay_buf >> 2U) & 1U);  // TAIL
  set_gpio_output(GPIOA,  7, (relay_buf >> 3U) & 1U);  // HEAD
  set_gpio_output(GPIOB,  0, (relay_buf >> 4U) & 1U);  // HIBEAM
  set_gpio_output(GPIOB,  1, (relay_buf >> 5U) & 1U);
  set_gpio_output(GPIOB, 10, (relay_buf >> 6U) & 1U);
  set_gpio_output(GPIOB, 12, (relay_buf >> 7U) & 1U);
}

uint8_t get_inputs(void){
  uint8_t input_buf = 0;
  input_buf |= get_gpio_input(GPIOC, 2) << 0U; // GPIOC2
  input_buf |= get_gpio_input(GPIOC, 3) << 1U;
  input_buf |= get_gpio_input(GPIOC, 4) << 2U;
  input_buf |= get_gpio_input(GPIOC, 5) << 3U;
  input_buf |= get_gpio_input(GPIOC, 6) << 4U;
  input_buf |= get_gpio_input(GPIOC, 7) << 5U;
  input_buf |= get_gpio_input(GPIOC, 8) << 6U;
  input_buf |= get_gpio_input(GPIOC, 9) << 7U;
  return input_buf;
}

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
      uint8_t value_0 = dat[1];
      bool enable = ((dat[3] >> 7) & 1U) != 0U;
      uint8_t index = dat[3] & COUNTER_CYCLE;
      if (dat[0] == lut_checksum(dat, CAN_SIZE, crc8_lut_1d)) {
        if (((current_index + 1U) & COUNTER_CYCLE) == index) {
          timeout = 0;
          state = NO_FAULT;
          if (enable) {
            can_lights = value_0;
          } else {
            // clear the fault state if values are 0 and the incoming request is valid
            if (value_0 == 0U) {
              state = NO_FAULT;
            } else {
              state = FAULT_INVALID;
            }
            can_lights = 0;
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

void TIM3_IRQ_Handler(void) {

  // check timer for sending the IO state and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[CAN_SIZE];
    dat[1] = (~gpio_lights >> 0) & 0xFFU;
    dat[2] = (~gpio_lights >> 8) & 0xFFU;
    dat[3] = ((state & 0xFU) << 4) | pkt_idx;
    dat[0] = lut_checksum(dat, CAN_SIZE, crc8_lut_1d);
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

  #ifdef DEBUG
  puts("STATE: ");
  puth(state);
  puts(" GPIO: ");
  puth(gpio_lights);
  puts(" CAN: ");
  puth(can_lights);
  puts("\n");
  #endif

}

// ***************************** main code *****************************
#define TPS_THRES 20
void loop(void) {
  gpio_lights = get_inputs();

  if (state != NO_FAULT) {
    can_lights = 0;
  }

  uint8_t lights = ~gpio_lights | can_lights;

  set_relays(~lights);

  watchdog_feed();
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)

  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

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

  // setup GPIO inputs
  set_gpio_mode(GPIOC, 2, MODE_INPUT);
  set_gpio_mode(GPIOC, 3, MODE_INPUT);
  set_gpio_mode(GPIOC, 4, MODE_INPUT);
  set_gpio_mode(GPIOC, 5, MODE_INPUT);
  set_gpio_mode(GPIOC, 6, MODE_INPUT);
  set_gpio_mode(GPIOC, 7, MODE_INPUT);
  set_gpio_mode(GPIOC, 8, MODE_INPUT);
  set_gpio_mode(GPIOC, 9, MODE_INPUT);
  // set_gpio_pullup(GPIOC, 2, PULL_UP);
  // set_gpio_pullup(GPIOC, 3, PULL_UP);
  // set_gpio_pullup(GPIOC, 4, PULL_UP);
  // set_gpio_pullup(GPIOC, 5, PULL_UP);
  // set_gpio_pullup(GPIOC, 6, PULL_UP);
  // set_gpio_pullup(GPIOC, 7, PULL_UP);
  // set_gpio_pullup(GPIOC, 8, PULL_UP);
  // set_gpio_pullup(GPIOC, 9, PULL_UP);

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

  set_gpio_output(GPIOB, 0, 0);

  set_relays(1);

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);
  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    loop();
  }

  return 0;
}
