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
#define IBST_USB
#define DEBUG

#ifdef IBST_USB
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

#ifdef IBST_USB

#include "ibst/can.h"

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

// OUTPUTS
// 0x38B
#define P_EST_MAX 0
#define P_EST_MAX_QF 1
#define VEHICLE_QF 1
#define IGNITION_ON 0
uint8_t current_speed = 0;

// 0x38C
#define P_LIMIT_EXTERNAL 120
#define Q_TARGET_DEFAULT 0x7e00 // this is the zero point
uint16_t q_target_ext = Q_TARGET_DEFAULT;
bool q_target_ext_qf = 0;

// 0x38D
#define P_TARGET_DRIVER 0
#define P_TARGET_DRIVER_QF 0
#define ABS_ACTIVE 0
#define P_MC 0
#define P_MC_QF 1

// INPUTS
uint16_t rel_input = 0;
uint16_t pos_input = 0;
bool pid_enable = 0;
bool rel_enable = 0;

// 0x38E
uint16_t output_rod_target = 0;
bool driver_brake_applied = 0;
bool brake_applied = 0;
bool brake_ok = 0;

// 0x38F
uint8_t ibst_status;
uint8_t ext_req_status;

// COUNTERS
uint8_t can1_count_out = 0;
uint8_t can1_count_in;
uint8_t can2_count_out_1 = 0;
uint8_t can2_count_out_2 = 0;
uint8_t can2_count_in_1;
uint8_t can2_count_in_2;
uint8_t can2_count_in_3;

#define MAX_TIMEOUT 50U
uint32_t timeout = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
#define FAULT_COUNTER 7U

uint8_t state = FAULT_STARTUP;

#define NO_EXTFAULT1 0U
#define EXTFAULT1_CHECKSUM1 1U
#define EXTFAULT1_CHECKSUM2 2U
#define EXTFAULT1_CHECKSUM3 3U
#define EXTFAULT1_SCE 4U
#define EXTFAULT1_COUNTER1 5U
#define EXTFAULT1_COUNTER2 6U
#define EXTFAULT1_COUNTER3 7U
#define EXTFAULT1_TIMEOUT 8U
#define EXTFAULT1_SEND1 9U
#define EXTFAULT1_SEND2 10U
#define EXTFAULT1_SEND3 11U

uint8_t can2state = NO_EXTFAULT1;

#define NO_EXTFAULT2 0U
#define EXTFAULT2_CHECKSUM1 1U
#define EXTFAULT2_CHECKSUM2 2U
#define EXTFAULT2_SCE 3U
#define EXTFAULT2_COUNTER1 4U
#define EXTFAULT2_COUNTER2 5U
#define EXTFAULT2_TIMEOUT 6U

const uint8_t crc_poly = 0x1D;  // standard crc8 SAE J1850
uint8_t crc8_lut_1d[256];

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
      case 0x20E: ;
        //uint64_t data; //sendESP_private2
        //uint8_t *dat = (uint8_t *)&data;
        uint8_t dat[6];
        for (int i=0; i<6; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        uint8_t index = dat[1] & COUNTER_CYCLE;
        if(dat[0] == lut_checksum(dat, 6, crc8_lut_1d)) {
          if (((can1_count_in + 1U) & COUNTER_CYCLE) == index) {
            //if counter and checksum valid accept commands
            pid_enable = ((dat[1] >> 5U) & 1U);
            rel_enable = ((dat[1] >> 4U) & 1U);
            pos_input = ((dat[5] & 0xFU) << 8U) | dat[4];
            rel_input = ((dat[3] << 8U) | dat[2]);
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
          puts("checksum fail 0x20E \n");
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
      case 0x366: ;
        uint8_t dat2[4];
        for (int i=0; i<4; i++) {
          dat2[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        if(dat2[0] == lut_checksum(dat2, 4, crc8_lut_1d)) {
          current_speed = dat2[3];
        }
        else {
          state = FAULT_BAD_CHECKSUM;
          puts("checksum fail 0x366 \n");
          puts("DATA: ");
          for(int ii = 0; ii < 4; ii++){
            puth2(dat2[ii]);
          }
          puts("\n");
          puts("expected: ");
          puth2(lut_checksum(dat2, 4, crc8_lut_1d));
          puts(" got: ");
          puth2(dat2[0]);
          puts("\n");
        }
      default: ;
    }
    can_rx(0);
    // next
    // CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN1);
  llcan_clear_send(CAN1);
}

void CAN2_RX0_IRQ_Handler(void) {
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN2->sFIFOMailBox[0].RIR >> 21;
    #ifdef DEBUG_CAN
    puts("CAN2 RX: ");
    puth(address);
    puts("\n");
    #endif
    switch (address) {
    /*  case 0x391:
        uint8_t dat[5];
        for (int i=0; i<5; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        uint64_t *data = (uint8_t *)&dat;
        uint8_t index = dat[6] & COUNTER_CYCLE;
        if(dat[0] = lut_checksum(dat, 8, crc8_lut_1d)) {
          if (((can2_count_in1 + 1U) & COUNTER_CYCLE) == index) {
            //if counter and checksum valid accept commands
            ebr_mode = (dat[1] >> 4) & 0x7;
            ebr_system_mode = (data >> 15) & 0x7;
            can2_count_in1++;
          }
          else {
            state = EXTFAULT1_COUNTER1;
          }
        }
        else {
          state = EXTFAULT1_CHECKSUM1;
        }
        break;*/
      case 0x38E: ;
        uint8_t dat[8]; //IBST_private1
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN2->sFIFOMailBox[0], i);
        }
        uint8_t index = dat[1] & COUNTER_CYCLE;
        if(dat[0] == lut_checksum(dat, 8, crc8_lut_1d)) {
          if (((can2_count_in_1 + 1U) & COUNTER_CYCLE) == index) {
            //if counter and checksum valid accept commands
            output_rod_target = ((dat[4] & 0xFU) << 8U) | dat[3];
            can2_count_in_1++;
          }
          else {
            can2state = EXTFAULT1_COUNTER2;
          }
        }
        else {
          can2state = EXTFAULT1_CHECKSUM2;
        }
        break;
      case 0x38F: ;
        uint64_t data2; //IBST_private2
        uint8_t *dat2 = (uint8_t *)&data2;
        for (int i=0; i<8; i++) {
          dat2[i] = GET_BYTE(&CAN2->sFIFOMailBox[0], i);
        }
        uint8_t index2 = dat2[1] & COUNTER_CYCLE;
        if(dat2[0] == lut_checksum(dat2, 8, crc8_lut_1d)) {
          if (((can2_count_in_3 + 1U) & COUNTER_CYCLE) == index2) {
            //if counter and checksum valid accept commands
            ibst_status = (data2 >> 19) & 0x7;
            driver_brake_applied = ((dat2[2] & 0x1) | (!((dat2[2] >> 1) & 0x1))); //Sends brake applied if ibooster says brake applied or if there's a fault with the brake sensor, assumes worst case scenario
            brake_applied = (driver_brake_applied | (output_rod_target > 0x23FU));
            can2_count_in_3++;
          }
          else {
            can2state = EXTFAULT1_COUNTER3;
          }
        }
        else {
          can2state = EXTFAULT1_CHECKSUM3;
        }
        break;
      default: ;
    }
    // next
    can_rx(1);
  }
}

void CAN2_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN2);
  llcan_clear_send(CAN2);
}

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
  state = FAULT_SCE;
  can_sce(CAN3);
  llcan_clear_send(CAN3);
}

// q_target_ext values
// 7e00 max rod return
// 8200 max brake.. TODO: check this?

// position values
// BC0 max rod position (3008)
// EC0 min rod position (-320)

#define P 2 // brake_rel is 2x brake_pos scale
#define I 0
#define D 0

#define OUTMAX 0x9200  //+-40ml/s
#define OUTMIN 0x6A00

uint16_t last_input;
int32_t output_sum;
int16_t error;

int to_signed(int d, int bits) {
  int d_signed = d;
  if (d >= (1 << MAX((bits - 1), 0))) {
    d_signed = d - (1 << MAX(bits, 0));
  }
  return d_signed;
}

void TIM3_IRQ_Handler(void) {

  if(pid_enable & !rel_enable) { //run PID loop
    q_target_ext_qf = 1;

    int q_target_out = Q_TARGET_DEFAULT; // this value is the zero point.

    error = to_signed(pos_input, 16) - to_signed(output_rod_target, 16); // position error
    q_target_out += (error * P);

    if (q_target_out > OUTMAX){
      q_target_out = OUTMAX;
    }

    if (q_target_out < OUTMIN){
      q_target_out = OUTMIN;
    }

    q_target_ext = q_target_out;

  }

  if (rel_enable && !pid_enable) { //relative mode
    q_target_ext_qf = 1;
    q_target_ext = rel_input;
  }

  if ((!rel_enable && !pid_enable) || (rel_enable && pid_enable)){ //both are 0 or both are 1
    q_target_ext_qf = 0;
    q_target_ext = Q_TARGET_DEFAULT;
    rel_enable = 0;
    pid_enable = 0;
    state = FAULT_INVALID;
  }

  if (brake_applied) { // handle relay
    set_gpio_output(GPIOB, 13, 1);
  } else {
    set_gpio_output(GPIOB, 13, 0);
  }

  if (driver_brake_applied){ // reset values
    q_target_ext_qf = 0;
    q_target_ext = Q_TARGET_DEFAULT;
  }

  // cmain loop for sending 100hz messages
  if ((CAN2->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8]; //sendESP_private3
    uint16_t pTargetDriver = P_TARGET_DRIVER * 4;
    dat[2] = pTargetDriver & 0xFFU;
    dat[3] = (pTargetDriver & 0x3U) >> 8;
    dat[4] = 0x0;
    dat[5] = 0x0;
    dat[6] = (uint8_t) P_MC_QF << 5;
    dat[7] = 0x0;
    dat[1] = can2_count_out_1;
    dat[0] = lut_checksum(dat, 8, crc8_lut_1d);

    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
    to_send.RDTR = 8;
    to_send.RIR = (0x38D << 21) | 1U;
    can_send(&to_send, 1, false);

  }
  else {
    // old can packet hasn't sent!
    state = EXTFAULT1_SEND1;
    #ifdef DEBUG_CAN
      puts("CAN2 MISS1\n");
    #endif
  }
  if ((CAN2->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) {
    uint8_t dat[8]; //sendESP_private2
    uint16_t p_limit_external = P_LIMIT_EXTERNAL * 2;

    dat[1] = can2_count_out_1 & COUNTER_CYCLE;
    dat[2] = p_limit_external & 0xFF;
    dat[3] = ((p_limit_external >> 8U) & 0x1U) | (q_target_ext & 0xFU) << 4U;
    dat[4] = (q_target_ext >> 4U) & 0xFF;
    dat[5] = ((q_target_ext >> 12U) & 0xFU) | (q_target_ext_qf << 4U); // what is ESP_diagnosticESP?
    dat[6] = 0x00;
    dat[7] = 0x00;
    dat[0] = lut_checksum(dat, 8, crc8_lut_1d);

    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
    to_send.RDTR = 8;
    to_send.RIR = (0x38C << 21) | 1U;
    can_send(&to_send, 1, false);;

    can2_count_out_1++;
    can2_count_out_1 &= COUNTER_CYCLE;

  }
  else {
    // old can packet hasn't sent!
    state = EXTFAULT1_SEND2;
    #ifdef DEBUG_CAN
      puts("CAN2 MISS2\n");
    #endif
  }
  if (!sent){
    if ((CAN2->TSR & CAN_TSR_TME2) == CAN_TSR_TME2) {
      uint8_t dat[8]; //sendESP_private1 every 20ms
      uint16_t ESP_vehicleSpeed = (((current_speed*16) / 9) & 0x3FFF);

      dat[1] = can2_count_out_2 & COUNTER_CYCLE;
      dat[2] = P_EST_MAX;
      dat[3] = P_EST_MAX_QF | (ESP_vehicleSpeed & 0x3FU);
      dat[4] = (ESP_vehicleSpeed >> 6U) & 0xFF;
      dat[5] = VEHICLE_QF | (IGNITION_ON << 3U);
      dat[6] = 0x00;
      dat[7] = 0x00;
      dat[0] = lut_checksum(dat, 8, crc8_lut_1d);

      CAN_FIFOMailBox_TypeDef to_send;
      to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
      to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
      to_send.RDTR = 8;
      to_send.RIR = (0x38B << 21) | 1U;
      can_send(&to_send, 1, false);

      can2_count_out_2++;
      can2_count_out_2 &= COUNTER_CYCLE;

    }
    else {
      // old can packet hasn't sent!
      state = EXTFAULT1_SEND3;
      #ifdef DEBUG_CAN
        puts("CAN2 MISS3\n");
      #endif
    }
  }
  sent = !sent;


  //send to EON
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[5];
    brake_ok = (ibst_status == 0x7);

    dat[4] = (can2state & 0xFU) << 4;
    dat[3] = (output_rod_target >> 8U) & 0x3FU;
    dat[2] = (brake_ok) | (driver_brake_applied << 1U) | (brake_applied << 2U) | (output_rod_target & 0x3FU);
    dat[1] = ((state & 0xFU) << 4) | can1_count_out;
    dat[0] = lut_checksum(dat, 5, crc8_lut_1d);

    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4];
    to_send.RDTR = 5;
    to_send.RIR = (0x20F << 21) | 1U;
    can_send(&to_send, 0, false);

    can1_count_out++;
    can1_count_out &= COUNTER_CYCLE;

  }
  else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG_CAN
      puts("CAN1 MISS1\n");
    #endif
  }
  // blink the LED

  TIM3->SR = 0;

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
    q_target_ext_qf = 0;
    q_target_ext = Q_TARGET_DEFAULT;
    pid_enable = 0;
    rel_enable = 0;
  } else {
    timeout += 1U;
  }

  #ifdef DEBUG
  puts("MODE: ");
  puth((pid_enable << 1U) | rel_enable);
  puts(" BRAKE_REQ: ");
  if (((pid_enable << 1U) | rel_enable) == 2){
    puth(pos_input);
    puts(" BRAKE_POS_ERR: ");
    puth(error);
  } else {
    puth(q_target_ext_qf);
  }
  puts(" BRAKE_POS: ");
  puth(output_rod_target);
  puts(" Q_TARGET_EXT: ");
  puth(q_target_ext);
  puts("\n");
  #endif
}

// ***************************** main code *****************************


void ibst(void) {
  // read/write
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
  #ifdef IBST_USB
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
  UNUSED(ret);
  ret = llcan_init(CAN2);
  UNUSED(ret);
  ret = llcan_init(CAN3);
  UNUSED(ret);

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 7);
  NVIC_EnableIRQ(TIM3_IRQn);

  // power on ibooster. needs to power on AFTER sending CAN to prevent ibst state from being 4
  set_gpio_mode(GPIOB, 12, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 12, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOB, 12, 1);

  //Brake switch relay
  set_gpio_mode(GPIOB, 13, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 13, OUTPUT_TYPE_PUSH_PULL);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    ibst();
  }

  return 0;
}