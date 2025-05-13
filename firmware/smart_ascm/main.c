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

// uncomment for usb debugging via debug_console.py
#define TGW_USB
// #define DEBUG_CAN
#define DEBUG_CTRL

#ifdef TGW_USB
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

#ifdef TGW_USB

#include "smart_ascm/can.h"

// ********************* usb debugging *********************
// TODO: neuter this if we are not debugging
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

#define CAN_UPDATE  0xF0 //bootloader

void CAN1_TX_IRQ_Handler(void) {
  process_can(0);
}

void CAN2_TX_IRQ_Handler(void) {
  process_can(1);
}

void CAN3_TX_IRQ_Handler(void) {
  process_can(2);
}

#define MAX_TIMEOUT 50U
uint32_t timeout_315 = 0;
uint32_t timeout_1f1 = 0;

#define NO_FAULT 0U
#define FAULT_SCE 1U
#define FAULT_STARTUP 2U

uint8_t state1 = FAULT_STARTUP;
uint8_t state2 = FAULT_STARTUP;
uint8_t state3 = FAULT_STARTUP;

bool send = 0;

#define BRAKE_CONTROL 0x315
bool op_ctrl_mode = 0;

#define IGN_CONTROL 0x1F1
bool gm_ign_mode = 0;

void CAN1_RX0_IRQ_Handler(void) { // Car chassis to ASCM
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN1->sFIFOMailBox[0].RIR | 1; // TXQ
    to_fwd.RDTR = CAN1->sFIFOMailBox[0].RDTR;
    to_fwd.RDLR = CAN1->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN1->sFIFOMailBox[0].RDHR;

    #ifdef DEBUG_CAN
    uint32_t address = (CAN1->sFIFOMailBox[0].RIR >> 21);
    uint8_t ide = (CAN1->sFIFOMailBox[0].RIR >> 2) & 0x01;
    if(ide){
      address = (CAN1->sFIFOMailBox[0].RIR >> 3);
    }
    puts("CAN1 RX: ");
    puth(address);
    puts("\n");
    #endif

    state1 = NO_FAULT;

    // send to CAN3
    can_send(&to_fwd, 2, false);
    // next
    can_rx(0);
    // CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state1 = FAULT_SCE;
  can_sce(CAN1);
  llcan_clear_send(CAN1);
}


void CAN2_RX0_IRQ_Handler(void) { // Car PT expansion with new 0x315 payload
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN2->sFIFOMailBox[0].RIR | 1; // TXQ
    to_fwd.RDTR = CAN2->sFIFOMailBox[0].RDTR;
    to_fwd.RDLR = CAN2->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN2->sFIFOMailBox[0].RDHR;

    uint32_t address = (CAN2->sFIFOMailBox[0].RIR >> 21);
    uint8_t ide = (CAN2->sFIFOMailBox[0].RIR >> 2) & 0x01;
    if(ide){
      address = (CAN2->sFIFOMailBox[0].RIR >> 3);
    }
    
    #ifdef DEBUG_CAN
    puts("CAN2 RX: ");
    puth(address);
    puts("\n");
    #endif

    state2 = NO_FAULT;

    switch (address) {
      case CAN_UPDATE:
        if (GET_BYTES_04(&CAN2->sFIFOMailBox[0]) == 0xdeadface) {
          if (GET_BYTES_48(&CAN2->sFIFOMailBox[0]) == 0x0ab00b1e) {
            enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
            NVIC_SystemReset();
          } else if (GET_BYTES_48(&CAN2->sFIFOMailBox[0]) == 0x02b00b1e) {
            enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
            NVIC_SystemReset();
          } else {
            puts("Failed entering Softloader or Bootloader\n");
          }
        }
        break;
      case BRAKE_CONTROL: // EBCMFrictionBrakeCmd
        op_ctrl_mode = 1;
        // reset the timer
        timeout_315 = 0;
        break;

      case IGN_CONTROL: // SystemPowerMode
        gm_ign_mode = 1;
        // reset the timer
        timeout_1f1 = 0;
        to_fwd.RIR &= 0xFFFFFFFE; // do not fwd
        break;

      default:
        to_fwd.RIR &= 0xFFFFFFFE; // do not fwd
        break;
    }
    // send to CAN1
    can_send(&to_fwd, 0, false);
    // next
    can_rx(1);
  }
}

void CAN2_SCE_IRQ_Handler(void) {
  state2 = FAULT_SCE;
  can_sce(CAN2);
  llcan_clear_send(CAN2);
}

void CAN3_RX0_IRQ_Handler(void) { // ASCM to car chassis (minus 0x315 when present on bus 2)
  // From the ASCM
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN3->sFIFOMailBox[0].RIR | 1; // TXQ
    to_fwd.RDTR = CAN3->sFIFOMailBox[0].RDTR;
    to_fwd.RDLR = CAN3->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN3->sFIFOMailBox[0].RDHR;

    uint32_t address = (CAN3->sFIFOMailBox[0].RIR >> 21);
    uint8_t ide = (CAN3->sFIFOMailBox[0].RIR >> 2) & 0x01;
    if(ide){
      address = (CAN3->sFIFOMailBox[0].RIR >> 3);
    }
    
    #ifdef DEBUG_CAN
    puts("CAN2 RX: ");
    puth(address);
    puts("\n");
    #endif

    state3 = NO_FAULT;

    switch (address) {
      case BRAKE_CONTROL: // EBCMFrictionBrakeCmd
        if (op_ctrl_mode) {
          to_fwd.RIR &= 0xFFFFFFFE; // do not fwd  
        }
        // FWD as-is
        break;
      default:
        // FWD as-is
        break;
    }
    // send to CAN1
    can_send(&to_fwd, 0, false);
    // next
    can_rx(2);
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  state3 = FAULT_SCE;
  can_sce(CAN3);
  llcan_clear_send(CAN3);
}

void TIM3_IRQ_Handler(void) {

  //send to EON. cap this to 50Hz
  if (send && gm_ign_mode){
    if ((CAN2->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
      uint8_t dat[4];
      dat[0] = ((state1 & 0xFU) << 4) | (op_ctrl_mode);
      dat[1] = ((state2 & 0xFU) << 4);
      dat[2] = ((state3 & 0xFU) << 4);
      dat[3] = 0;

      CAN_FIFOMailBox_TypeDef to_send;
      to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
      to_send.RDTR = 4;
      to_send.RIR = (0x2FF << 21) | 1U;
      can_send(&to_send, 1, false);

    }
    else {
      // old can packet hasn't sent!
      #ifdef DEBUG_CAN
        puts("CAN2 MISS1\n");
      #endif
    }
  }

  send = !send;

  // up timeouts
  if (timeout_315 == MAX_TIMEOUT){
    op_ctrl_mode = 0;
  } else {
    timeout_315 += 1U;
  }

  if (timeout_1f1 == MAX_TIMEOUT){
    gm_ign_mode = 0;
  } else {
    timeout_1f1 += 1U;
  }
  
  TIM3->SR = 0;

#ifdef DEBUG_CTRL
puts("op_ctrl_mode: ");
puth2(op_ctrl_mode);
puts("\n");
#endif
}

// ***************************** main code *****************************


void gw(void) {
  // read/write
  // maybe we can implement the ADC and DAC here for pedal functionality?
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

  // init microsecond system timer
  // increments 1000000 times per second
  // generate an update to set the prescaler
  TIM2->PSC = 48-1;
  TIM2->CR1 = TIM_CR1_CEN;
  TIM2->EGR = TIM_EGR_UG;
  // use TIM2->CNT to read

  // init board
  current_board->init();
  // enable USB
  #ifdef TGW_USB
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  usb_init();
  #endif

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
  ret = llcan_init(CAN2);
  ret = llcan_init(CAN3);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 7);
  NVIC_EnableIRQ(TIM3_IRQn);

  // TODO: figure out a function for these GPIOs
  // set_gpio_mode(GPIOB, 12, MODE_OUTPUT);
  // set_gpio_output_type(GPIOB, 12, OUTPUT_TYPE_PUSH_PULL);
  // set_gpio_output(GPIOB, 12, 1);

  // set_gpio_mode(GPIOB, 13, MODE_OUTPUT);
  // set_gpio_output_type(GPIOB, 13, OUTPUT_TYPE_PUSH_PULL);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main loop
  while (1) {
    gw();
  }

  return 0;
}
