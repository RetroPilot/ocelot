// ********************* Includes *********************
#include "../config.h"
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

// addresses
#define OUTPUT_ADDRESS 0x22F
#define INPUT_ADDRESS 0x22E

// uncomment for usb debugging via debug_console.py
#define DEBUG_USB
// #define DEBUG
// #define DEBUG_CAN

// functions
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#define max(a,b)            (((a) > (b)) ? (a) : (b))

#ifdef DEBUG_USB
  #include "drivers/uart.h"
  #include "drivers/usb.h"
#else
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
#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

#ifdef DEBUG_USB

#include "gpio_example/can.h"

// ********************* usb debugging *********************
void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
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
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

#endif

// ***************************** can port *****************************
#define CAN_UPDATE  0x341 //bootloader
#define COUNTER_CYCLE 0xFU

void CAN1_TX_IRQ_Handler(void) {
  process_can(0);
}

void CAN2_TX_IRQ_Handler(void) {
  // CAN2->TSR |= CAN_TSR_RQCP0;
  process_can(1);
}

void CAN3_TX_IRQ_Handler(void) {
  process_can(2);
}

bool sent;

#define MAX_TIMEOUT 50U
uint32_t timeout = 0;
uint32_t current_index = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
#define FAULT_COUNTER 7U

uint8_t state = FAULT_STARTUP;

const uint8_t crc_poly = 0x1D;  // standard crc8 SAE J1850
uint8_t crc8_lut_1d[256];

int16_t torque_input = 0;
int16_t angle_input = 0;
bool steer_ok = 0;

// CAN 1 read function
// this is where you read in data from CAN 1 and set variables
void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN1->sFIFOMailBox[0].RIR >> 21;
    #ifdef DEBUG_CAN
    puts("CAN1 RX: ");
    puth(address);
    puts("\n");
    #endif
    switch (address) {
      case CAN_UPDATE:
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
        break;

      case INPUT_ADDRESS: ;
        uint8_t dat[6];
        for (int i=0; i<6; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        if(dat[0] == lut_checksum(dat, 6, crc8_lut_1d)) {
          int16_t angle_req = (dat[3] << 8) | dat[2];
          int16_t torque_req = (dat[5] << 8) | dat[4];
          //uint8_t mode = ((dat[1] >> 4) & 2U);
          uint8_t index = dat[1] & COUNTER_CYCLE;
          if (((current_index + 1U) & COUNTER_CYCLE) == index) {
            //if counter and checksum valid accept commands
            steer_ok = 1;
            #ifdef DEBUG
              puts("torque req: ");
              puth(torque_req);
              puts("\n");
            #endif
            // TODO: implement enable / mode
            if (1) {
              torque_input = torque_req;
            } else {
              // not enabled, everything is 0
              torque_input = 0;
              angle_input = 0;
              // clear the fault state if values are 0
              if ((torque_req == 0U) && (angle_req == 0U)) {
                state = NO_FAULT;
              } else {
                state = FAULT_INVALID;
              }
              }
              current_index = index;
          } else {
            state = FAULT_COUNTER;
          }
          state = NO_FAULT;
          timeout = 0;
        }
        else {
          state = FAULT_BAD_CHECKSUM;
          puts("checksum fail 0x22E \n");
        }
        break;

      default: 
      break;
    }
    can_rx(0);
    // next
    CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
}
void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN1);
  llcan_clear_send(CAN1);
}

// CAN 2 read function
void CAN2_RX0_IRQ_Handler(void) {
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN2->sFIFOMailBox[0].RIR >> 21;
    #ifdef DEBUG_CAN
    puts("CAN2 RX: ");
    puth(address);
    puts("\n");
    #else
    UNUSED(address);
    #endif
    // next
    can_rx(1);
  }
}
void CAN2_SCE_IRQ_Handler(void) {
  // state = FAULT_SCE;
  // can_sce(CAN2);
  llcan_clear_send(CAN2);
}

// CAN 3 read function
void CAN3_RX0_IRQ_Handler(void) {
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN3->sFIFOMailBox[0].RIR >> 21;
    #ifdef DEBUG_CAN
    puts("CAN1 RX: ");
    puth(address);
    puts("\n");
    #else
    UNUSED(address);
    #endif
    // next
    can_rx(2);
  }
}
void CAN3_SCE_IRQ_Handler(void) {
  // state = FAULT_SCE;
  // can_sce(CAN3);
  llcan_clear_send(CAN3);
}
// uint8_t gpio = 1;
// timer 3 interrupt. Use this function to perform tasks at specific intervals. see main() for details
uint8_t pkt_idx = 0;
void TIM3_IRQ_Handler(void) {
  #ifdef DEBUG
    puth(TIM3->CNT);
    puts("\n");
  #endif
  // set_gpio_output(GPIOB, 12, gpio);
  // gpio = !gpio;

  // send to EON
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[7];
    dat[6] = (torque_input >> 8) & 0xFF;
    dat[5] = (torque_input >> 0) & 0xFF;
    dat[4] = (0 >> 0) & 0xFF;
    dat[3] = (0 >> 8) & 0xFF;
    dat[2] = ((0 >> 0) | steer_ok) & 0xFF;
    dat[1] = ((state & 0xFU) << 4) | pkt_idx;
    dat[0] = lut_checksum(dat, 7, crc8_lut_1d);

    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16);
    to_send.RDTR = 7;
    to_send.RIR = (OUTPUT_ADDRESS << 21) | 1U;
    can_send(&to_send, 0, false);
    pkt_idx++;
    pkt_idx &= COUNTER_CYCLE;
  }
  else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG_CAN
      puts("CAN1 MISS1\n");
    #endif
  }

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }

  // reset the timer
  TIM3->SR = 0;

}

// ***************************** main code *****************************

// This function is the main application. It is run in a while loop from main() and is interrupted by those specified in main().
void loop(void) {
  // read/write
  if (state == NO_FAULT) {
    // clip input
    torque_input = min(torque_input, 1024);
    torque_input = max(torque_input, -1024);

    register_set(&(TIM5->CCR3), (1024+torque_input), 0xFFFFU);
    register_set(&(TIM5->CCR4), (1024-torque_input), 0xFFFFU);
  } else {
    register_set(&(TIM5->CCR3), 1024, 0xFFFFU);
    register_set(&(TIM5->CCR4), 1024, 0xFFFFU);
  }
  watchdog_feed();
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  // REGISTER_INTERRUPT(CAN2_TX_IRQn, CAN2_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  // REGISTER_INTERRUPT(CAN2_RX0_IRQn, CAN2_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  // REGISTER_INTERRUPT(CAN2_SCE_IRQn, CAN2_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  // REGISTER_INTERRUPT(CAN3_TX_IRQn, CAN3_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  // REGISTER_INTERRUPT(CAN3_RX0_IRQn, CAN3_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  // REGISTER_INTERRUPT(CAN3_SCE_IRQn, CAN3_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)

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
  #ifdef DEBUG_USB
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  usb_init();
  #endif

  puts("INIT");

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan1 speed");
  }
  // llcan_speed_set = llcan_set_speed(CAN2, 5000, false, false);
  // if (!llcan_speed_set) {
  //   puts("Failed to set llcan2 speed");
  // }
  // llcan_speed_set = llcan_set_speed(CAN3, 5000, false, false);
  // if (!llcan_speed_set) {
  //   puts("Failed to set llcan3 speed");
  // }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);
  // ret = llcan_init(CAN2);
  // UNUSED(ret);
  // ret = llcan_init(CAN3);
  // UNUSED(ret);

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);
  
  // ############### TIMER INTERRUPTS ###############
  // clock / (psc+1)*(arr) = TIM3_IRQ interrupt frequency
  // in this case 48,000,000 / ((7+1)*(59999+1)) = 100Hz
  uint8_t psc = 7;
  uint16_t arr = 59999;
  register_set(&(TIM3->PSC), (psc), 0xFFFFU);
  register_set(&(TIM3->ARR), (arr), 0xFFFFU);
  register_set(&(TIM3->DIER), TIM_DIER_UIE, 0x5F5FU);
  register_set(&(TIM3->CR1), TIM_CR1_CEN, 0x3FU);
  TIM3->SR = 0;
  NVIC_EnableIRQ(TIM3_IRQn);

  // ##################### GPIO #####################
  // in this example, we will use PB12 and PB13

  // setup GPIO
  set_gpio_mode(GPIOB, 12, MODE_OUTPUT);
  set_gpio_mode(GPIOB, 13, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 12, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 13, OUTPUT_TYPE_PUSH_PULL);
  // set GPIO state ON
  set_gpio_output(GPIOB, 12, 1);
  set_gpio_output(GPIOB, 13, 1);

  // ##################### PWM #####################
  // in this example, we will use GPIOs PA2 and PA3
  // they map to TIM4_CH3 and TIM5_CH4, respectively.

  // init PWM clock
  // frequency = clk/(psc+1)*(arr+1)
  register_set(&(TIM5->PSC), (0), 0xFFFFU);
  register_set(&(TIM5->CR1), TIM_CR1_CEN, 0x3FU);     // Enable

  // set up alternate functions for PA2 and PA3
  set_gpio_alternate(GPIOA, 2, GPIO_AF2_TIM5);
  set_gpio_alternate(GPIOA, 3, GPIO_AF2_TIM5);

  // init the timer
  pwm_init(TIM5, 3);
  pwm_init(TIM5, 4);

  // change the reload register value. 2048 gives us 11 bit resolution @ 24kHz
  register_set(&(TIM5->ARR), (2048-1), 0xFFFFFFFFU);
  
  // set PWM to 50% (ARR / 1024)
  register_set(&(TIM5->CCR3), 1024, 0xFFFFU);
  register_set(&(TIM5->CCR4), 1024, 0xFFFFU);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main loop
  while (1) {
    loop();
  }

  return 0;
}