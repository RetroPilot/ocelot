// ********************* Includes *********************
#include "../config.h"
#include "libc.h" // Assuming libc.h provides memset

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

#include "obj/gitversion.h"

#include "drivers/uart.h"
#include "chimera/usb.h"
#include "chimera/can.h"
#include "chimera/chimera.h"

extern int _app_start[0xc000];

// uncomment for usb debugging via debug_console.py
#define DEBUG
// #define DEBUG_CAN

// functions
#define min(a,b)            (((a) < (b)) ? (a) : (b))

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}
void __attribute__ ((noinline)) enable_fpu(void) {
  // enable the FPU
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
  FPU->FPCCR |= (FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk);
}

volatile uint32_t vss_pulse_count = 0;
volatile uint32_t cps_pulse_count = 0;
volatile uint32_t vss_dt_us = 0;
volatile uint32_t cps_dt_us = 0;
volatile uint32_t vss_last_us = 0;
volatile uint32_t cps_last_us = 0;
uint16_t vss = 0;

uint32_t adc[2];
uint32_t adc_cmp = 0;

bool controls_allowed = false;
bool relay_on = 0;
bool ignition_on = 0;

bool ignition_line = 0;
bool relay_req_obdc = 0;
bool relay_req_usb = 0;
bool brake_pressed = 0;

uint32_t steer_angle_rate = 0;
uint32_t steer_angle = 0;
uint32_t eng_rpm = 0;
bool brake_pressed_can = 0;

uint32_t last_rpm = 0;
static uint32_t last_revolution_time = 0;
static uint32_t last_pulse_time = 0;
static uint8_t cached_rel_cnt = 0;
static uint32_t last_pulse_count = 0; 
uint16_t crank_angle = 0; 

// can be read from CAN or ADC, depending on flash configuration
bool buttons[4];

uint8_t ignition_cnt = 0;

struct __attribute__((packed)) health_t {
  uint32_t uptime_pkt;
  uint32_t voltage_pkt;
  uint32_t current_pkt;
  uint32_t can_rx_errs_pkt;
  uint32_t can_send_errs_pkt;
  uint32_t can_fwd_errs_pkt;
  uint32_t gmlan_send_errs_pkt;
  uint32_t faults_pkt;
  uint8_t ignition_line_pkt;
  uint8_t ignition_can_pkt;
  uint8_t controls_allowed_pkt;
  uint8_t gas_interceptor_detected_pkt;
  uint8_t car_harness_status_pkt;
  uint8_t usb_power_mode_pkt;
  uint8_t safety_mode_pkt;
  int16_t safety_param_pkt;
  uint8_t fault_status_pkt;
  uint8_t power_save_enabled_pkt;
};

// ********************* usb debugging *********************
int get_health_pkt(void *dat) {
  COMPILE_TIME_ASSERT(sizeof(struct health_t) <= MAX_RESP_LEN);
  struct health_t * health = (struct health_t*)dat;

  health->uptime_pkt = uptime_cnt;
  health->voltage_pkt = 0;
  health->current_pkt = 0;

  health->ignition_line_pkt = (uint8_t)ignition_line;
  health->ignition_can_pkt = (uint8_t)(ignition_can);

  health->controls_allowed_pkt = (uint8_t)(controls_allowed);
  health->gas_interceptor_detected_pkt = 0;
  health->can_rx_errs_pkt = can_rx_errs;
  health->can_send_errs_pkt = can_send_errs;
  health->can_fwd_errs_pkt = can_fwd_errs;
  health->gmlan_send_errs_pkt = 0;
  health->car_harness_status_pkt = car_harness_status;
  health->usb_power_mode_pkt = usb_power_mode;
  health->safety_mode_pkt = 17;
  health->safety_param_pkt = 0;
  health->power_save_enabled_pkt = 0;

  health->fault_status_pkt = fault_status;
  health->faults_pkt = faults;

  return sizeof(*health);
}

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

// callback function to handle flash writes over USB
void usb_cb_ep0_out(void *usbdata_ep0, int len_ep0) {
  if ((usbdata_ep0 == last_usb_data_ptr) & (len_ep0 == sizeof(flash_config_t))) {
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
  // Clear the buffer after processing for this endpoint
  memset(usbdata, 0, len);
}

// send on CAN
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  int dpkt = 0;
  uint32_t *d32 = (uint32_t *)usbdata;
  for (dpkt = 0; dpkt < (len / 4); dpkt += 4) {
    CAN_FIFOMailBox_TypeDef to_push;
    to_push.RDHR = d32[dpkt + 3];
    to_push.RDLR = d32[dpkt + 2];
    to_push.RDTR = d32[dpkt + 1];
    to_push.RIR = d32[dpkt];

    uint8_t bus_number = (to_push.RDTR >> 4) & CAN_BUS_NUM_MASK;
    can_send(&to_push, bus_number, false);
  }
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
  unsigned int resp_len = 0;
  UNUSED(hardwired);
  uart_ring *ur = NULL;

  // Store the setup packet and data pointer for deferred processing if needed
  // This must be done for all commands that might involve an OUT data phase
  // and need deferred processing.
  memcpy(&last_setup_pkt, setup, sizeof(USB_Setup_TypeDef));
  last_usb_data_ptr = usbdata; // Save the pointer to the buffer

  switch (setup->b.bRequest) {
    /*
    case 0xb4:
      puts("Setting Relay\n");
      switch (setup->b.wValue.w) {
        case 0:
          set_gpio_output(GPIOB, 0, 0);
          break;
        case 1:
          set_gpio_output(GPIOB, 0, 1);
          break;
        default:
          puts("Invalid GPIO value\n");
          break;
      }
      puth(setup->b.wValue.w);
      break;
    case 0xb5:
    puts("Setting OBD\n");
      switch (setup->b.wValue.w) {
        case 0:
          set_gpio_output(GPIOB, 1, 0);
          break;
        case 1:
          set_gpio_output(GPIOB, 1, 1);
          break;
        default:
          puts("Invalid GPIO value\n");
          break;
      }
      break;
      */
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
    // 0xd2: Get health/status (required by panda API)
    case 0xd2:
      resp_len = get_health_pkt(resp);
      break;
    // **** 0xd3: get first 64 bytes of signature
    case 0xd3:
      {
        resp_len = 64;
        char * code = (char*)_app_start;
        int code_len = _app_start[0];
        (void)memcpy(resp, &code[code_len], resp_len);
      }
      break;
    // **** 0xd4: get second 64 bytes of signature
    case 0xd4:
      {
        resp_len = 64;
        char * code = (char*)_app_start;
        int code_len = _app_start[0];
        (void)memcpy(resp, &code[code_len + 64], resp_len);
      }
      break;
    // **** 0xd6: get version
    case 0xd6:
      COMPILE_TIME_ASSERT(sizeof(gitversion) <= MAX_RESP_LEN);
      (void)memcpy(resp, gitversion, sizeof(gitversion));
      resp_len = sizeof(gitversion) - 1U;
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    case 0xdc:
      // set safety mode
      if (setup->b.wValue.w == 0 || setup->b.wValue.w == 3 || setup->b.wValue.w == 19){
        relay_req_usb = 0;
      } else {
        relay_req_usb = 1;
      }
      break;
    // 0xde: Set CAN bitrate (optional, or can remove if bitrate fixed)
    case 0xde:
      if (setup->b.wValue.w == 0) {
        can_speed[0] = setup->b.wIndex.w;
        can_init(0);
      }
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
    // 0xe5: Set CAN loopback (optional)
    case 0xe5:
      can_loopback = (setup->b.wValue.w > 0U);
      can_init_all();
      break;

    // 0xf1: Clear CAN RX ring buffer (optional)
    case 0xf1:
      if (setup->b.wValue.w == 0 || setup->b.wValue.w == 0xFFFFU) {
        can_clear(&can_rx_q);
      }
      break;
    case 0xf3: {
      heartbeat_counter = 0U;
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
      // Ignore unsupported requests
      break;
  }
  return resp_len;
}

// ***************************** can port *****************************
#define CAN_UPDATE  0x341 //bootloader
#define COUNTER_CYCLE 0xFU

void CAN1_TX_IRQ_Handler(void) {
  process_can(0);
  CAN1->TSR |= CAN_TSR_RQCP0;
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

// CAN 1 read function
// this is where you read in data from CAN 1 and set variables
void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN1->sFIFOMailBox[0].RIR >> 21;
    #ifdef DEBUG_CAN
    puts("CAN1 RX: ");
    puth(address);
    puts("\n");
    #else
    UNUSED(address);
    #endif
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
  // can_sce(CAN2);
  llcan_clear_send(CAN2);
}

// CAN 3 read function
void CAN3_RX0_IRQ_Handler(void) {
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN3->sFIFOMailBox[0].RIR >> 21;
    const flash_config_t *cfg = NULL;

    cfg = signal_configs[1]; // STEER_ANGLE
    if (cfg && (cfg->can.enabled & 1) && (address == cfg->can.can_id) && cfg->cfg_type == CFG_TYPE_CAN) {
      steer_angle = extract_scaled_signal(cfg, &CAN3->sFIFOMailBox[0]);
    }

    cfg = signal_configs[2]; // STEER_ANGLE_FRAC
    if (cfg && (cfg->can.enabled & 1) && (address == cfg->can.can_id) && cfg->cfg_type == CFG_TYPE_CAN) {
      steer_angle += extract_scaled_signal(cfg, &CAN3->sFIFOMailBox[0]);
    }

    cfg = signal_configs[3]; // STEER_ANGLE_RATE
    if (cfg && (cfg->can.enabled & 1) && (address == cfg->can.can_id) && cfg->cfg_type == CFG_TYPE_CAN) {
      steer_angle_rate = extract_scaled_signal(cfg, &CAN3->sFIFOMailBox[0]);
    }

    cfg = signal_configs[4]; // VEHICLE_SPEED
    if (cfg && (cfg->can.enabled & 1) && (address == cfg->can.can_id) && cfg->cfg_type == CFG_TYPE_CAN) {
      vss = extract_scaled_signal(cfg, &CAN3->sFIFOMailBox[0]);
    }

    cfg = signal_configs[5]; // IGNITION_CAN
    if (cfg && (cfg->can.enabled & 1) && (address == cfg->can.can_id) && cfg->cfg_type == CFG_TYPE_CAN) {
      ignition_can = extract_scaled_signal(cfg, &CAN3->sFIFOMailBox[0]) & 1U;
    }

    cfg = signal_configs[6]; // ENG_RPM
    if (cfg && (cfg->can.enabled & 1) && (address == cfg->can.can_id) && cfg->cfg_type == CFG_TYPE_CAN) {
      eng_rpm = extract_scaled_signal(cfg, &CAN3->sFIFOMailBox[0]) & 1U;
    }

    cfg = signal_configs[7]; // BRAKE_PRESSED
    if (cfg && (cfg->can.enabled & 1) && (address == cfg->can.can_id) && cfg->cfg_type == CFG_TYPE_CAN) {
      brake_pressed_can = extract_scaled_signal(cfg, &CAN3->sFIFOMailBox[0]) & 1U;
    }

    can_rx(2);
    CAN3->RF0R |= CAN_RF0R_RFOM0;  // release FIFO
  }
}
void CAN3_SCE_IRQ_Handler(void) {
  // can_sce(CAN3);
  llcan_clear_send(CAN3);
}

static void send_can1_message(CAN_FIFOMailBox_TypeDef *to_send, uint8_t *pkt_idx) {
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    CAN1->sTxMailBox[0].TDTR = to_send->RDTR;
    CAN1->sTxMailBox[0].TDLR = to_send->RDLR;
    CAN1->sTxMailBox[0].TDHR = to_send->RDHR;
    CAN1->sTxMailBox[0].TIR = to_send->RIR | CAN_TI0R_TXRQ;
  } else if ((CAN1->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) {
    CAN1->sTxMailBox[1].TDTR = to_send->RDTR;
    CAN1->sTxMailBox[1].TDLR = to_send->RDLR;
    CAN1->sTxMailBox[1].TDHR = to_send->RDHR;
    CAN1->sTxMailBox[1].TIR = to_send->RIR | CAN_TI1R_TXRQ;
  } else if ((CAN1->TSR & CAN_TSR_TME2) == CAN_TSR_TME2) {
    CAN1->sTxMailBox[2].TDTR = to_send->RDTR;
    CAN1->sTxMailBox[2].TDLR = to_send->RDLR;
    CAN1->sTxMailBox[2].TDHR = to_send->RDHR;
    CAN1->sTxMailBox[2].TIR = to_send->RIR | CAN_TI2R_TXRQ;
  } else {
    state = FAULT_SEND;
    // puts(miss_tag);
    // puts("CAN1 TSR: ");
    // puth(CAN1->TSR);
    // puts("\n");
    return;
  }

  *pkt_idx = (*pkt_idx + 1) & COUNTER_CYCLE;
}

// timer 3 interrupt. Use this function to perform tasks at specific intervals. see main() for details
uint8_t pkt_idx1 = 0;
uint8_t pkt_idx2= 0;
uint8_t pkt_idx3 = 0;
uint8_t pkt_idx4 = 0;
uint8_t turn = 0;
void TIM3_IRQ_Handler(void) {
  CAN_FIFOMailBox_TypeDef to_send;
  const flash_config_t *cfg;
  const flash_config_t *sys_cfg = signal_configs[0];

  switch (turn) {
    case 0: { // steer_angle
      uint8_t dat[6];
      cfg = signal_configs[1];
      dat[5] = (((cfg->can.enabled & 1) & 0xFU) << 4) | pkt_idx1;
      dat[4] = (steer_angle_rate >> 8) & 0xFF;
      dat[3] = (steer_angle_rate >> 0) & 0xFF;
      dat[2] = (steer_angle >> 8) & 0xFF;
      dat[1] = (steer_angle >> 0) & 0xFF;
      dat[0] = lut_checksum(dat, 6, crc8_lut_1d);

      to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
      to_send.RDHR = dat[4] | (dat[5] << 8);
      to_send.RDTR = 6;
      to_send.RIR = (80 << 21) | 1U;

      if (sys_cfg && sys_cfg->sys.can_out_en & 1){
        send_can1_message(&to_send, &pkt_idx1);
      }
      break;
    }

    case 1: { // ENGINE
      uint8_t dat[8];
      
      dat[7] = ((state & 0xFU) << 4) | pkt_idx2;
      dat[6] = (eng_rpm >> 8) & 0xFF;
      dat[5] = (eng_rpm >> 0) & 0xFF;
      dat[4] = ((brake_pressed | brake_pressed_can) & 1) << 1 | ((ignition_line | ignition_can) & 1);
      dat[3] = (cps_pulse_count >> 0) & 0xFF;
      dat[2] = (cps_dt_us >> 8) & 0xFF;
      dat[1] = (cps_dt_us >> 0) & 0xFF;
      dat[0] = lut_checksum(dat, 8, crc8_lut_1d);

      to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
      to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
      to_send.RDTR = 8;
      to_send.RIR = (117 << 21) | 1U;

      if (sys_cfg && sys_cfg->sys.can_out_en & 2) {
        send_can1_message(&to_send, &pkt_idx2);
      }
      break;
    }

    case 2: {
      //VSS signal 
      uint8_t dat[8];
      cfg = signal_configs[3];
      dat[7] = ((cfg->can.enabled & 1) << 4) | pkt_idx3;
      dat[6] = (vss >> 8) & 0xFF;
      dat[5] = (vss >> 0) & 0xFF;
      dat[4] = 0;
      dat[3] = (vss_pulse_count >> 0) & 0xFF;
      dat[2] = (vss_dt_us >> 8) & 0xFF;
      dat[1] = (vss_dt_us >> 0) & 0xFF;
      dat[0] = lut_checksum(dat, 8, crc8_lut_1d);

      to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
      to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
      to_send.RDTR = 8;
      to_send.RIR = (118 << 21) | 1U;

      if (sys_cfg && sys_cfg->sys.can_out_en & 4){ 
        send_can1_message(&to_send, &pkt_idx3);
      }
      break;
    }

    case 3: { // CRUISE
      uint8_t dat[8];
      dat[7] = ((state & 0xFU) << 4) | pkt_idx4;
      dat[6] = (adc[1] >> 8) & 0xFF;
      dat[5] = (adc[1] >> 0) & 0xFF;
      dat[4] = (adc[0] >> 8) & 0xFF;
      dat[3] = (adc[0] >> 0) & 0xFF;
      dat[2] = ((buttons[0] << 0) | (buttons[1] << 1) | (buttons[2] << 2) | (buttons[3]) << 3) & 0xF;
      dat[1] = 0;
      dat[0] = lut_checksum(dat, 8, crc8_lut_1d);

      to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
      to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
      to_send.RDTR = 8;
      to_send.RIR = (256 << 21) | 1U;

      if (sys_cfg && sys_cfg->sys.can_out_en & 8){ 
        send_can1_message(&to_send, &pkt_idx4);
      }
      break;
      }
    default: {
      break;
    }
  }

  turn = (turn + 1) % 5;

  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }

  TIM3->SR = 0;
}


// called at 8Hz
uint8_t loop_counter = 0U;
void TIM1_BRK_TIM9_IRQ_Handler(void) {
  if (TIM9->SR != 0) {
    // decimated to 1Hz
    if(loop_counter == 0U){
      // increase heartbeat counter and cap it at the uint32 limit
      if (heartbeat_counter < __UINT32_MAX__) {
        heartbeat_counter += 1U;
      }
      // if the heartbeat has been gone for a while, go to SILENT safety mode and enter power save
      if (heartbeat_counter > 1) {
        // puts("heartbeat not seen for 0x");
        // puth(heartbeat_counter);
        // puts(" seconds. Turning off 12V and IGN.\n");
        relay_req_usb = 0;
      }
      // check registers
      check_registers();
      uptime_cnt += 1U;
      uptime_cnt &= 0xFFFF;
    }
    loop_counter++;
    loop_counter %= 8U;
    if(ignition_line | ignition_can){
      if(ignition_cnt < 40){ // 5 sec 
        ignition_cnt++;
      }
    } else {
      ignition_cnt = 0;
      relay_on = 0;
    }
  }
  // VSS and CPS timeout checks
  const uint32_t VSS_TIMEOUT_US = 900000;
  const uint32_t CPS_TIMEOUT_US = 10000;
  uint32_t now = TIM2->CNT;
  if ((now - vss_last_us) > VSS_TIMEOUT_US) {
    vss_dt_us = 0;
  }
  if ((now - cps_last_us) > CPS_TIMEOUT_US) {
    cps_dt_us = 0;  
  }

  adc[0] = adc_get(12); // ADC1
  adc[1] = adc_get(13);  // ADC2
  for (int i = 0; i <= 3; i++) {
    const flash_config_t *cfg = signal_configs[i + 11];
    if (!cfg || cfg->cfg_type != CFG_TYPE_ADC || !(cfg->adc.adc_en & 1)) continue;

    uint8_t ch = cfg->adc.adc_num;
    if (ch >= 2) continue;

    uint32_t adc_cmp = adc[ch];
    uint32_t ref = (ch == 0) ? cfg->adc.adc1 : cfg->adc.adc2;

    uint32_t lo = (ref >= cfg->adc.adc_tolerance) ? (ref - cfg->adc.adc_tolerance) : 0;
    uint32_t hi = ref + cfg->adc.adc_tolerance;

    buttons[i] = (adc_cmp >= lo && adc_cmp <= hi);

    cfg = signal_configs[0];
    if (cfg && (cfg->sys.debug_lvl & 1)){
      puts("BTN "); puth(i);
      puts(": adc["); puth(ch); puts("]="); puth(adc_cmp);
      puts(" ref="); puth(ref);
      puts(" lo="); puth(lo); puts(" hi="); puth(hi);
      puts(" => "); puts(buttons[i] ? "ON\n" : "OFF\n");
    }
  }

  const flash_config_t *syscfg = NULL;
  syscfg = signal_configs[0];
  // Debug Level 1
  if (syscfg && (syscfg->sys.debug_lvl & 1)){
    syscfg = signal_configs[1];
    puts("STEER_ANGLE_MAJ: ");
    puts("confgured: ");
    puts((signal_configs[1]->can.enabled & 1) ? "true" : "false");
    puts(" STEER_ANGLE_MIN: ");
    puts((signal_configs[2]->can.enabled & 1) ? "true" : "false");
    puts(" steer angle: ");
    puth(steer_angle);
    puts("\n");

    puts("ADC0 RAW: ");
    puth(adc[0]);
    puts(" ADC1 RAW: ");
    puth(adc[1]);
    puts("\n");

    puts("IGN: ");
    puth(ignition_line | ignition_can);
    puts("\n");

    puts("VSS config type: ");
    puth(signal_configs[4]->cfg_type);
    puts("vss: ");
    puth(vss);
    puts(" vss_pulse: ");
    puth(vss_dt_us);
    puts("\n");
    puts("engine rpm: ");
    puth(eng_rpm);
    puts("\n");
  }

  // for (int i=0; i<4; i++){
  //   puth(buttons[i]);
  //   puts("\n");
  // }
  // puth(adc_cmp);
  // puts("\n");
  TIM9->SR = 0;
}

// ******************************** VSS ********************************
void EXTI4_IRQ_Handler(void) {
  if (EXTI->PR & EXTI_PR_PR4) {
    EXTI->PR = EXTI_PR_PR4;

    vss_pulse_count = (vss_pulse_count + 1) & 0xFFF;

    uint32_t now = TIM2->CNT;
    vss_dt_us = (now - vss_last_us);  // truncation is okay
    vss_last_us = now;
  }
}
// ******************************** CPS ********************************
void EXTI9_5_IRQ_Handler(void) {
  if (EXTI->PR & EXTI_PR_PR5) {
    EXTI->PR = EXTI_PR_PR5;

    cps_pulse_count = (cps_pulse_count + 1);
    
    // Clip to cached reluctor count for fast interrupt handling
    if (cached_rel_cnt > 0) {
      cps_pulse_count %= cached_rel_cnt;
    } else {
      cps_pulse_count &= 0xFFF; // fallback to 12-bit counter
    }

    uint32_t now = TIM2->CNT;
    cps_dt_us = (uint16_t)(now - cps_last_us);  // truncation is okay
    cps_last_us = now;
  }
}

// vss kph helper function
uint32_t last_vss = 0;
uint32_t compute_vss_kph(uint32_t pulse_width_us, uint32_t vss_ppm, uint8_t vss_type) {
    if (pulse_width_us == 0) {
        return 0;
    }

    float pulse_width_s = (float)pulse_width_us / 1000000.0f;
    float frequency_hz = 1.0f / pulse_width_s;
    float speed_kph = ((frequency_hz * 3600) / vss_ppm) * 100; //* 160.9;

    if (!(vss_type & 1U)){
      speed_kph = speed_kph * 1.6;
    }

    if (last_vss > 0) {
      if (vss > last_vss * 2 || vss < last_vss / 2) {
        return last_vss;
      }
    }

    last_vss = (uint32_t)speed_kph;
    return (uint32_t)speed_kph;
}

uint32_t compute_eng_rpm(uint32_t current_time_us, uint32_t pulse_count, uint32_t pulse_width_us, uint8_t rel_cnt, uint8_t skipped_teeth) {
    if (rel_cnt == 0) {
        return 0; 
    }

    uint32_t rpm = 0;
    uint32_t teeth_per_rev = rel_cnt - skipped_teeth;
    bool revolution_complete = false;
    
    // Detect revolution completion
    if (skipped_teeth > 0) {
        // Gap detection method - pulse width > 1.5x previous pulse
        if (pulse_width_us > (last_pulse_time + (last_pulse_time >> 1)) && last_pulse_time > 0) {
            revolution_complete = true;
            crank_angle = 0; // Reset at TDC
        }
        last_pulse_time = pulse_width_us;
        crank_angle = (pulse_count * 360) / teeth_per_rev;
    } else {
        // Pulse count wrap method - detect when count wraps from max back to 0
        revolution_complete = (pulse_count < last_pulse_count);
        crank_angle = 0;
    }
    
    last_pulse_count = pulse_count;
    
    // Calculate RPM on revolution completion
    if (revolution_complete && last_revolution_time > 0) {
        uint32_t revolution_time = current_time_us - last_revolution_time;
        
        // Handle timer rollover (happens every ~4294 seconds)
        if (current_time_us < last_revolution_time) {
            revolution_time = (0xFFFFFFFF - last_revolution_time) + current_time_us;
        }
        
        // Sanity check - revolution time should be reasonable (10ms to 10s)
        if (revolution_time > 10000 && revolution_time < 10000000) {
            rpm = 60000000UL / revolution_time;
        }
    }
    
    // Update revolution timestamp
    if (revolution_complete) {
        last_revolution_time = current_time_us;
    }
    
    // Debug output for troubleshooting
    static uint32_t debug_counter = 0;
    if (++debug_counter % 1000 == 0) { // Print every 1000 calls
        puts("RPM Debug: pc="); puth(pulse_count);
        puts(" lpc="); puth(last_pulse_count);
        puts(" rc="); puth(revolution_complete);
        puts(" lrt="); puth(last_revolution_time);
        puts(" rpm="); puth(rpm);
        puts("\n");
    }

    // Simple filtering
    if (rpm > 0 && last_rpm > 0) {
        if (rpm > last_rpm + 200 || rpm < last_rpm - 200) {
            return last_rpm;
        }
    }

    if (rpm > 0) {
        last_rpm = rpm;
    }
    return rpm;
}


// ***************************** main code *****************************

// This function is the main application. It is run in a while loop from main() and is interrupted by those specified in main().
void loop(void) {

  // read/write
  ignition_line = !get_gpio_input(GPIOA, 3);
  relay_req_obdc = !get_gpio_input(GPIOA, 2);
  brake_pressed = !get_gpio_input(GPIOA, 6);

  if (ignition_line | ignition_can){
    ignition_on = 1; 
    relay_on=1;
    // if (relay_req_obdc | relay_req_usb){ //(ignition_cnt < 40)
    //   relay_on=1;
    // } else {
    //   relay_on=0;
    // }
  } else {
    ignition_on = 0;
    relay_on = 0;
  }

  set_gpio_output(GPIOB, 0, relay_on);
  set_gpio_output(GPIOB, 1, !ignition_on);

  const flash_config_t *cfg = NULL;
  cfg = signal_configs[4];
  if (cfg && cfg->cfg_type == CFG_TYPE_HALL) {    
    vss = compute_vss_kph(vss_dt_us, (uint32_t) cfg->hall.vss_ppd, (uint8_t) cfg->hall.is_km);
  }

  cfg = signal_configs[6];
  if (cfg && cfg->cfg_type == CFG_TYPE_HALL) {
    // Cache reluctor count for fast interrupt access
    cached_rel_cnt = cfg->hall.rel_cnt;
    eng_rpm = compute_eng_rpm(TIM2->CNT, cps_pulse_count, cps_dt_us, (uint8_t) cfg->hall.rel_cnt, (uint8_t) cfg->hall.skipped_teeth);

  }

  watchdog_feed();
}

int main(void) {

  // ######################## FLASH HANDLING ###########################
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
  }
  // ###################################################################

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

  REGISTER_INTERRUPT(EXTI9_5_IRQn, EXTI9_5_IRQ_Handler, 10000U, (1 << 22U))  // CPS
  REGISTER_INTERRUPT(EXTI4_IRQn, EXTI4_IRQ_Handler, 10000U, (1 << 23U))      // VSS

  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  // 8Hz timer
  REGISTER_INTERRUPT(TIM1_BRK_TIM9_IRQn, TIM1_BRK_TIM9_IRQ_Handler, 10U, FAULT_INTERRUPT_RATE_TIM9)

  // setup microsecond timer
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC = 48-1;
  TIM2->ARR = 0xFFFFFFFF;
  TIM2->DIER |= TIM_DIER_UIE;
  TIM2->CR1 = TIM_CR1_CEN;
  TIM2->EGR = TIM_EGR_UG;
  // use TIM2->CNT to read
  // reset with TIM2->CNT = 0;

  //enable fpu
  enable_fpu();

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

  adc_init();

  puts("INIT\n");

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

  // 8hz
  timer_init(TIM9, 183);
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

  // interrupts for VSS and CPS
  // PA4 and PA5, respectively
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI4 | SYSCFG_EXTICR2_EXTI5);
  EXTI->IMR |= EXTI_IMR_IM4 | EXTI_IMR_IM5;
  EXTI->FTSR |= EXTI_FTSR_TR4 | EXTI_FTSR_TR5;
  EXTI->PR = EXTI_PR_PR4 | EXTI_PR_PR5;
  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_EnableIRQ(EXTI9_5_IRQn); // handles lines 5-9

  // GPIO
  set_gpio_mode(GPIOB, 0, MODE_OUTPUT); // RELAY
  set_gpio_mode(GPIOB, 1, MODE_OUTPUT); // IGN_OBDC
  set_gpio_mode(GPIOA, 2, MODE_INPUT);  // RELAY_REQ
  set_gpio_mode(GPIOA, 3, MODE_INPUT);  // IGNITION
  set_gpio_mode(GPIOA, 4, MODE_INPUT);  // VSS
  set_gpio_mode(GPIOA, 5, MODE_INPUT);  // CPS
  set_gpio_mode(GPIOA, 6, MODE_INPUT);  // BRAKE_PED
  set_gpio_mode(GPIOC, 2, MODE_ANALOG); // ADC1
  set_gpio_mode(GPIOC, 3, MODE_ANALOG); // ADC2

  set_gpio_pullup(GPIOB, 0, PULL_DOWN);
  set_gpio_pullup(GPIOB, 1, PULL_UP);
  set_gpio_pullup(GPIOA, 4, PULL_UP);
  set_gpio_pullup(GPIOA, 5, PULL_UP);
  set_gpio_pullup(GPIOA, 3, PULL_UP);
  set_gpio_pullup(GPIOA, 2, PULL_UP);
  set_gpio_pullup(GPIOA, 6, PULL_UP);

  // set_gpio_pullup(GPIOC, 2, PULL_UP);
  // set_gpio_pullup(GPIOC, 3, PULL_UP);

  set_gpio_output(GPIOB, 0, 0); // RELAY off
  set_gpio_output(GPIOB, 1, 1); // IGN_OBDC off

  RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN;

  if (signal_configs[0] != NULL && (signal_configs[0]->sys.iwdg_en & 1)){
    puts("WATCHDOG ENABLED\n");
    watchdog_init();
  } else {
    puts("WATCHDOG DISABLED\n");
  }

  uint32_t reason = RCC->CSR;
  puts("Reset cause: ");
  puth(reason);
  puts("\n");
  RCC->CSR |= RCC_CSR_RMVF;

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main loop
  while (1) {
    loop();
  }

  return 0;
}
