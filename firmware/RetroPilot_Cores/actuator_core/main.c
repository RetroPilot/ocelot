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

// TODO: handle the VSS sensor
// probably going to have to do a 1 second update interval

// #define CAN CAN1

#define PEDAL_USB
#define DEBUG

#ifdef PEDAL_USB
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
// control theory:
// set clutch/relay thing
// get to requested position 
// min 0x820
// max 0xe90
// max input = 1648.. lets call that 1500 (0x5DC)
#define MAX_INPUT 0x5DC
#define TPS_MIN 0x200
// addresses to be used on CAN
#define CAN_GAS_INPUT  0x400
#define CAN_GAS_OUTPUT 0x401U
#define CAN_OUTPUT_ACC_STATE 0x150
#define CAN_GAS_SIZE 6
#define COUNTER_CYCLE 0xFU

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN1->TSR |= CAN_TSR_RQCP0;
}

// acc states
bool acc_mode = 0;
bool acc_engaged = 0;
uint16_t acc_set_speed_kmh = 0;
// buttons
bool acc_on_off_sw = 0;
bool acc_speed_up = 0;
bool acc_speed_down = 0;
bool acc_cancel = 0;

#define TPS_THRES 20

// two independent values
uint32_t gas_set = 0;
// clutch
bool clutch = 0;

// vehicle speed
uint16_t vss = 0;
uint16_t vss_cnt = 0;

#define MAX_TIMEOUT 5U
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

uint32_t pdl0 = 0;
uint32_t pdl1 = 0;

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
            // TODO: we gotta get this outta here!
            set_gpio_output(GPIOB, 4, 1); // CLUTCH
            gas_set = value_0;
            uint32_t lower_deadzone = TPS_MIN + gas_set - TPS_THRES;
            uint32_t upper_deadzone = TPS_MIN + gas_set + TPS_THRES;
            set_gpio_output(GPIOB, 2, 0); // DIS
            set_gpio_output(GPIOA, 2, 1); // PWM
            set_gpio_output(GPIOB, 0, 1); // DIR
            // deadzone check
            if (pdl0 >= lower_deadzone && pdl0 <= upper_deadzone) {
              // Stop the motor if within the deadzone range
              set_gpio_output(GPIOA, 2, 0);
              set_gpio_output(GPIOB, 2, 1);
            }
            else if (pdl0 < (uint32_t)(TPS_MIN + gas_set)) {
                // Turn the motor forward
                set_gpio_output(GPIOA, 2, 0);
                set_gpio_output(GPIOA, 3, 1);
            }
            else if (pdl0 > (uint32_t)(TPS_MIN + gas_set)) {
                // Turn the motor reverse
                set_gpio_output(GPIOA, 2, 1);
                set_gpio_output(GPIOB, 0, 0);
            }
          } else {
            set_gpio_output(GPIOB, 4, 0); // CLUTCH
            set_gpio_output(GPIOA, 2, 0); // PWM
            set_gpio_output(GPIOB, 2, 0); // DIR
            set_gpio_output(GPIOB, 0, 1); // DIS
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

int led_value = 0;
uint32_t timer = 0;

bool send = 0;
void TIM3_IRQ_Handler(void) {

  // check timer for sending the user pedal and clearing the CAN
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    dat[1] = (pdl0 >> 0) & 0xFFU;
    dat[2] = (pdl0 >> 8) & 0xFFU;
    dat[3] = (vss >> 0) & 0xFFU;
    dat[4] = (vss >> 8) & 0xFFU;
    dat[5] = (vss_cnt >> 0) & 0xFFU;
    dat[6] = (vss_cnt >> 8) & 0xFFU;
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
  }
  if (send) {
    uint8_t dat[6];
    dat[5] = 0;
    dat[4] = (acc_set_speed_kmh & 0xFF);
    dat[3] = (acc_set_speed_kmh << 8U);
    dat[2] = (acc_mode << 5U) | (acc_engaged << 4U) | (acc_on_off_sw << 3U) | (acc_speed_up << 2U) | (acc_speed_down << 1U) | (acc_cancel);
    dat[1] = ((state & 0xFU) << 4) | pkt2_idx;
    dat[0] = lut_checksum(dat, 6, crc8_lut_1d);
    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4] | (dat[5] << 8);
    to_send.RDTR = 6;
    to_send.RIR = (CAN_OUTPUT_ACC_STATE << 21) | 1U;
    can_send(&to_send, 0, false);
    ++pkt2_idx;
    pkt2_idx &= COUNTER_CYCLE;
  } else { 
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      // puts("CAN MISS\n");
    #endif
  }

  // blink the LED
  current_board->set_led(LED_GREEN, led_value);
  led_value = !led_value;

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
    timeout += 1U;
  }

  #ifdef DEBUG
  puth(TIM3->CNT);
  puts(" PDL0: ");
  puth(pdl0);
  puts(" PDL1: ");
  puth(pdl1);
  puts(" VSS : ");
  puth(vss);
  puts(" VSS_CNT : ");
  puth(vss_cnt);
  puts(" buttons: ");
  puth((acc_mode << 5U) | (acc_engaged << 4U) | (acc_on_off_sw << 3U) | (acc_speed_up << 2U) | (acc_speed_down << 1U) | (acc_cancel));
  puts(" gas_set: ");
  puth(gas_set);
  puts(" clutch: ");
  puth(clutch);
  puts("\n");
  #endif

  // reset buttonstates
  acc_on_off_sw = 0;
  acc_cancel = 0;
  acc_speed_down = 0;
  acc_speed_up = 0;

}

// this should only be called if the ARR value is hit (no VSS pulse in 1 second or under 1 MPH)
void TIM2_IRQ_HANDLER(void) {
  puts("tim2 interrupt\n");
  if (TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~TIM_SR_UIF;  // Clear the interrupt flag
    TIM2->CNT = 0;
    timer = 0;
  }
  vss = 0;
  // EXTI->SWIER |= EXTI_SWIER_SWIER0;
}

void EXTI9_5_IRQ_Handler(void) {
  volatile unsigned int pr = EXTI->PR & (1U << 9);
  if ((pr & (1U << 9)) != 0U) {
    timer = TIM2->CNT;
    vss_cnt = (vss_cnt + 1) % 0xFFFFU;
    // reset timer and clear interrupt
    TIM2->CNT=0;
    EXTI->PR = (1U << 9);
  }
}

// ***************************** main code *****************************
void pedal(void) {

  pdl0 = adc_get(12); // TPS
  pdl1 = adc_get(13);  // CC_SW

  switch(pdl1){
    case 0x800 ... 0x900: ;
      //cancel
      acc_cancel = 1;
    break;
    case 0x400 ... 0x500: ;
      // - set
      acc_speed_down = 1;
    break;
    case 0x200 ... 0x300: ;
      // + res
      acc_speed_up = 1;
    break;
    case 0x00 ... 0x100: ;
     // on-off
     acc_on_off_sw = 1;
     break;
    default : ;
     break;
  }

  // calculate VSS
  // sensor produces 4 pulses / rotation
  // need to work out ratio of sensor / wheel to get speed

  vss =  timer;
  // vss_cnt = (3600000000.0f) / (4000.0f * (float)(timer));
  // vss = vss_cnt / 0.44704f;


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

  // TIM2 interrupt on ARR trigger
  REGISTER_INTERRUPT(TIM2_IRQn, TIM2_IRQ_HANDLER, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  REGISTER_INTERRUPT(EXTI9_5_IRQn, EXTI9_5_IRQ_Handler, 0xFFFF, FAULT_INTERRUPT_RATE_TACH)
  register_set(&(SYSCFG->EXTICR[3]), SYSCFG_EXTICR3_EXTI9_PA, 0x00U);
  register_set_bits(&(EXTI->IMR), (1U << 9));
  register_set_bits(&(EXTI->RTSR), (1U << 9));
  // register_set_bits(&(EXTI->FTSR), (1U << 9));
  NVIC_EnableIRQ(EXTI9_5_IRQn);

  // init board
  current_board->init();

  set_gpio_mode(GPIOC, 3, MODE_ANALOG);

#ifdef PEDAL_USB
  // enable USB
  usb_init();
#endif

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
  set_gpio_output(GPIOB, 4, 0); // clutch off
  // motors
  set_gpio_mode(GPIOA, 2,  MODE_OUTPUT); // PWM MOTOR1
  set_gpio_mode(GPIOB, 2,  MODE_OUTPUT); // DIS MOTOR1
  set_gpio_mode(GPIOB, 0,  MODE_OUTPUT); // DIR MOTOR1
  set_gpio_mode(GPIOA, 3,  MODE_OUTPUT); // PWM MOTOR2
  set_gpio_mode(GPIOB, 3,  MODE_OUTPUT); // DIS MOTOR2
  set_gpio_mode(GPIOB, 1,  MODE_OUTPUT); // DIR MOTOR2
  set_gpio_mode(GPIOA, 10, MODE_OUTPUT); // CSN
  set_gpio_output_type(GPIOA, 2,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOA, 3,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 0,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 1,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 2,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOB, 3,  OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output_type(GPIOA, 10, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOA, 2,  PULL_DOWN);
  set_gpio_pullup(GPIOA, 3,  PULL_DOWN);
  set_gpio_pullup(GPIOB, 0,  PULL_DOWN);
  set_gpio_pullup(GPIOB, 1,  PULL_DOWN);
  set_gpio_pullup(GPIOB, 2,  PULL_UP);
  set_gpio_pullup(GPIOB, 3,  PULL_UP);
  set_gpio_pullup(GPIOA, 10, PULL_UP); // CSN Active Low
  set_gpio_output(GPIOA, 2,  0);
  set_gpio_output(GPIOA, 3,  0);
  set_gpio_output(GPIOB, 0,  0);
  set_gpio_output(GPIOB, 1,  0);
  set_gpio_output(GPIOB, 2,  1); // Motor1 disabled at boot
  set_gpio_output(GPIOB, 3,  1); // Motor2 disabled at boot
  set_gpio_output(GPIOA, 10, 0);

  // vss calcultation
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

  NVIC_EnableIRQ(TIM2_IRQn);

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);
  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    pedal();
  }

  return 0;
}
