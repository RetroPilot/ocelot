/*
CUSTOM gateway implementation for the RSG OFFROAD Tundra-swapped 4-runner.
This board will convert the Toyota PTCAN signals to RetroPilot-compatible signals.
Additionally, this board handles the cruise control switch functions. Hopefully, 
it will also be able to switch "modes" by intercepting the stalk.
*/

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
#include "drivers/dac.h"
#include "drivers/timer.h"

#include "gpio.h"
#include "crc.h"

#include "drivers/uart.h"
#include "drivers/usb.h"

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

#include "rsg_gateway/can.h"

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

// ***************************** can port *****************************
#define CAN_UPDATE  0x23F //bootloader
#define COUNTER_CYCLE 0xFU
#define LKA_COUNTER_CYCLE = 0x3FU

void CAN1_TX_IRQ_Handler(void) {
  process_can(0);
}

void CAN2_TX_IRQ_Handler(void) {
  process_can(1);
}

void CAN3_TX_IRQ_Handler(void) {
  process_can(2);
}

bool sent;

// Toyota Checksum algorithm
uint8_t toyota_checksum(int addr, uint8_t *dat, int len){
  int cksum = 0;
  for(int ii = 0; ii < (len - 1); ii++){
    cksum = (cksum + dat[ii]); 
  }
  cksum += len;
  cksum += ((addr >> 8U) & 0xFF); // idh
  cksum += ((addr) & 0xFF); // idl
  return cksum & 0xFF;
}

#define MAX_TIMEOUT 50U
uint32_t timeout = 0;

// fault states
uint8_t state = 0;
#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
#define FAULT_COUNTER 7U

// CAN1 connects to the controls bus
#define CAN_OUTPUT_SAS 0x50
#define CAN_OUTPUT_WHEEL_SPEED 0x100
#define CAN_OUTPUT_ACC_STATE 0x150
#define CAN_INPUT_ACC_REQ 0x151

// CAN2 connects to the PTCAN of the car
#define CAN2_INPUT_SAS 0x25
#define CAN2_INPUT_WHEEL_SPEED1 0xB0
#define CAN2_INPUT_WHEEL_SPEED2 0xB2
#define CAN2_INPUT_SPEED 0xB4
#define CAN2_INPUT_BRAKE_MODULE 0x224

// can input counters
uint8_t can1_count_in = 0;
uint8_t can2_count_in = 0;

bool brakes_pressed = 0;

int16_t steer_angle = 0;
int16_t steer_rate = 0;

uint16_t wheel_speed_fl = 0;
uint16_t wheel_speed_fr = 0;
uint16_t wheel_speed_rr = 0;
uint16_t wheel_speed_rl = 0;

uint16_t vehicle_speed = 0;

// acc states
bool acc_mode = 0;
bool acc_engaged = 0;
uint16_t acc_set_speed_kmh = 0;
// buttons
bool acc_on_off_sw = 0;
bool acc_speed_up = 0;
bool acc_speed_down = 0;
bool acc_cancel = 0;

const uint8_t crc_poly = 0x1D;  // standard crc8 SAE J1850
uint8_t crc8_lut_1d[256];


// adc and dac stuff
uint32_t adc_in = 0;

void CAN2_RX0_IRQ_Handler(void) {
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN2->sFIFOMailBox[0].RIR >> 21;
    #ifdef DEBUG_CAN
    puts("CAN2 RX: ");
    puth(address);
    puts("\n");
    #endif
    switch (address) {
      case CAN_INPUT_ACC_REQ: ;
        uint8_t dat[6];
        for (int i=0; i<6; i++) {
          dat[i] = GET_BYTE(&CAN2->sFIFOMailBox[0], i);
        }
        uint8_t index = dat[1] & COUNTER_CYCLE;
        if(dat[0] == lut_checksum(dat, 6, crc8_lut_1d)) {
          if (((can1_count_in + 1U) & COUNTER_CYCLE) == index) {
            can1_count_in++;
          }
          else {
            state = FAULT_COUNTER;
          }
          state = NO_FAULT;
          timeout = 0;
        }
        else {
          state = FAULT_BAD_CHECKSUM;
          puts("checksum fail 0x22E \n");
          puts("DATA: ");
          for(int ii = 0; ii < 6; ii++){
            puth2(dat[ii]);
          }
          puts("\n");
          puts("expected: ");
          puth2(lut_checksum(dat, 6, crc8_lut_1d));
          puts(" got: ");
          puth2(dat[0]);
          puts("\n");
        }
        break;
      default: ;
    }
    can_rx(1);
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN1);
  llcan_clear_send(CAN1);
}

void CAN1_RX0_IRQ_Handler(void) {
  // PTCAN connects here
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN1->sFIFOMailBox[0].RIR >> 21;
    #ifdef DEBUG_CAN
    puts("CAN2 RX: ");
    puth(address);
    puts("\n");
    #endif
    switch (address) {
      case CAN2_INPUT_SAS: ;
      uint8_t dat[8];
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        steer_angle = (((dat[0] & 0xF) << 8) | dat[1]) + (dat[4] >> 4 & 0xF);
        steer_rate = ((dat[4] & 0xF) << 8 | dat[5]);
        break;
      case CAN2_INPUT_WHEEL_SPEED1: ;
      uint8_t dat1[8];
        for (int i=0; i<8; i++) {
          dat1[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        wheel_speed_fl = (dat1[0] << 8) | (dat1[1]);
        wheel_speed_fr = (dat1[2] << 8) | (dat1[3]);
        break;
      case CAN2_INPUT_WHEEL_SPEED2: ;
        uint8_t dat2[8];
        for (int i=0; i<8; i++) {
          dat2[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        wheel_speed_rl = (dat2[0] << 8) | dat2[1];
        wheel_speed_rr = (dat2[2] << 8) | dat2[3];
        break;
      case CAN2_INPUT_SPEED: ;
      uint8_t dat3[8];
        for (int i=0; i<8; i++) {
          dat3[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        vehicle_speed = (dat3[5] << 8) | dat3[6];
        break;
      case CAN2_INPUT_BRAKE_MODULE: ;
      uint8_t dat4[8];
        for (int i=0; i<8; i++) {
          dat4[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        brakes_pressed = (dat4[0] >> 4) & 0x01;      
      default: ;
    }
    can_rx(0);
  }
}

void CAN2_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN2);
  llcan_clear_send(CAN2);
}

void CAN3_RX0_IRQ_Handler(void) {
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {
    #ifdef DEBUG_CAN
    uint16_t address = CAN3->sFIFOMailBox[0].RIR >> 21;
    puts("CAN3 RX: ");
    puth(address);
    puts("\n");
    #endif
    // can_rx(2);
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN3);
  llcan_clear_send(CAN3);
}

int to_signed(int d, int bits) {
  int d_signed = d;
  if (d >= (1 << MAX((bits - 1), 0))) {
    d_signed = d - (1 << MAX(bits, 0));
  }
  return d_signed;
}

uint8_t can1_count_out = 0;
void TIM3_IRQ_Handler(void) {
  // cmain loop for sending 100hz messages

  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    // steer angle and steer rate
    dat[5] = (steer_rate & 0xFF);
    dat[4] = (steer_rate >> 8U);
    dat[3] = (steer_angle & 0xFF);
    dat[2] = (steer_angle >> 8U);
    dat[1] = ((state & 0xFU) << 4) | can1_count_out;
    dat[0] = lut_checksum(dat, 6, crc8_lut_1d);

    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4] | (dat[5] << 8);
    to_send.RDTR = 6;
    to_send.RIR = (CAN_OUTPUT_SAS << 21) | 1U;
    can_send(&to_send, 1, false);

    can1_count_out++;
    can1_count_out &= COUNTER_CYCLE;
  }
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    // wheel speeds
    dat[7] = (wheel_speed_rr & 0xFF);
    dat[6] = (wheel_speed_rr >> 8U);
    dat[5] = (wheel_speed_rl & 0xFF);
    dat[4] = (wheel_speed_rl >> 8U);
    dat[3] = (wheel_speed_fr & 0xFF);
    dat[2] = (wheel_speed_fr >> 8U);
    dat[1] = (wheel_speed_fl & 0xFF);
    dat[0] = (wheel_speed_fl >> 8U);

    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
    to_send.RDTR = 8;
    to_send.RIR = (CAN_OUTPUT_WHEEL_SPEED << 21) | 1U;
    can_send(&to_send, 1, false);
  }
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    dat[5] = 0;
    dat[4] = (acc_set_speed_kmh & 0xFF);
    dat[3] = (acc_set_speed_kmh << 8U);
    dat[2] = (acc_mode << 5U) | (acc_engaged << 4U) | (acc_on_off_sw << 3U) | (acc_speed_up << 2U) | (acc_speed_down << 1U) | (acc_cancel);
    dat[1] = ((state & 0xFU) << 4) | can1_count_out;
    dat[0] = lut_checksum(dat, 6, crc8_lut_1d);
    
    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4] | (dat[5] << 8);
    to_send.RDTR = 6;
    to_send.RIR = (CAN_OUTPUT_ACC_STATE << 21) | 1U;
    can_send(&to_send, 1, false);

  } else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG_CAN
      puts("CAN1 MISS1\n");
    #endif
  }
  // blink the LED

  TIM3->SR = 0;

  // up timeout
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }

  puts("SWITCHES: ");
  puth((acc_mode << 5U) | (acc_engaged << 4U) | (acc_on_off_sw << 3U) | (acc_speed_up << 2U) | (acc_speed_down << 1U) | (acc_cancel));
  puts("\n");
  // puts("SAS: ");
  // puth(steer_angle);
  // puts(" STEER_RATE: ");
  // puth(steer_rate);
  // puts(" SPEED: ");
  // puth(vehicle_speed);
  // puts("\n ");
}

// ***************************** main code *****************************


void loop(void) {
  // read/write
  adc_in = adc_get(12);

  //handle the ACC_SW
  switch(adc_in){
    case 0x700 ... 0x800: ;
      //cancel
      acc_cancel = 1;
    break;
    case 0xb00 ... 0xc00: ;
      // - set
      acc_speed_down = 1;
    break;
    case 0xd00 ... 0xe00: ;
      // + res
      acc_speed_up = 1;
    break;
    case 0xf00 ... 0xFFF: ;
     // on-off
     acc_on_off_sw = 1;
     break;
    default : ;
    acc_on_off_sw = 0;
    acc_cancel = 0;
    acc_speed_down = 0;
    acc_speed_up = 0;
  }
  
  watchdog_feed();

}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN2_TX_IRQn, CAN2_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_RX0_IRQn, CAN2_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_SCE_IRQn, CAN2_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN3_TX_IRQn, CAN3_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_RX0_IRQn, CAN3_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_SCE_IRQn, CAN3_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)

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
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  usb_init();

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan1 speed");
  }
  llcan_speed_set = llcan_set_speed(CAN2, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan2 speed");
  }
  llcan_speed_set = llcan_set_speed(CAN3, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan3 speed");
  }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);
  ret = llcan_init(CAN2);
  UNUSED(ret);
  ret = llcan_init(CAN3);
  UNUSED(ret);

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 7);
  NVIC_EnableIRQ(TIM3_IRQn);

  dac_init();
  adc_init();

  // turn on relay
  set_gpio_mode(GPIOB, 0, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 0, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOB, 0, 1);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    loop();
  }

  return 0;
}