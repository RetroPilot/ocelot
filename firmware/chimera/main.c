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
#include "chimera/chimera.h"

extern int _app_start[0xc000];

// addresses
#define OUTPUT_ADDRESS 0x201
#define INPUT_ADDRESS 0x200

// uncomment for usb debugging via debug_console.py
#define DEBUG
// #define DEBUG_CAN

// functions
#define min(a,b)            (((a) < (b)) ? (a) : (b))

#include "drivers/uart.h"
#include "chimera/usb.h" // Ensure this header defines necessary USB types/functions
#include "chimera/can.h"

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
volatile uint16_t vss_dt_us = 0;
volatile uint16_t cps_dt_us = 0;
volatile uint32_t vss_last_us = 0;
volatile uint32_t cps_last_us = 0;

bool controls_allowed = false;
uint8_t current_safety_mode = 17U;
int16_t current_safety_param = 0;
uint8_t relay_on = 0;
uint8_t ignition_on = 0;

bool flash_configured = 0;
uint32_t steer_angle = 0;
uint32_t vss_can = 0;

const can_signal_config_t *steer_angle_major_cfg = NULL;
const can_signal_config_t *steer_angle_minor_cfg = NULL;
const can_signal_config_t *vehicle_speed_cfg = NULL;

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

// ********************** flash write **********************
void flash_unlock(void);
void flash_lock(void);
void flash_erase_sector(uint8_t sector);
void flash_write_word(uint32_t addr, uint32_t val);
void flash_config_format(void);
uint32_t crc32(const void *data, size_t len);

#define CHIMERA
// Global/static variables to hold setup packet details for deferred processing
static USB_Setup_TypeDef last_setup_pkt;
static uint8_t *last_usb_data_ptr = NULL; // Points to the buffer provided by USB driver
volatile bool config_write_pending = false; // Flag to indicate a pending config write

// This function will handle the actual flash write.
// It should be called when EP0 OUT data transfer is complete.
void handle_deferred_config_write(void) {
  if (!config_write_pending) {
    return; // No pending write
  }

  // Clear the flag immediately to prevent re-entry
  config_write_pending = false;

  puts("Executing deferred config write.\n");

  if (last_setup_pkt.b.wLength.w == sizeof(can_signal_config_t) &&
      last_setup_pkt.b.wValue.w < MAX_CONFIG_ENTRIES &&
      last_usb_data_ptr != NULL) {

    can_signal_config_t *new_entry = (can_signal_config_t *)last_usb_data_ptr;
    config_block_t current;

    // Load existing config
    memcpy(&current, (void *)CONFIG_FLASH_ADDR, sizeof(config_block_t));

    // Update entry with the now-complete data
    current.entries[last_setup_pkt.b.wValue.w] = *new_entry;
    current.magic = FLASH_CONFIG_MAGIC;
    current.crc32 = crc32(&current.entries[0], sizeof(current.entries));

    // Write to flash
    flash_unlock();
    flash_erase_sector(CONFIG_FLASH_SECTOR);
    for (uint32_t i = 0; i < sizeof(config_block_t); i += 4) {
      flash_write_word(CONFIG_FLASH_ADDR + i,
                      *(uint32_t *)((uint8_t *)&current + i));
    }
    flash_lock();

    puts("Wrote config entry successfully.\n");

  }
  // Clear the usbdata buffer after the write is complete
  if (last_usb_data_ptr != NULL) {
    memset(last_usb_data_ptr, 0, last_setup_pkt.b.wLength.w);
    last_usb_data_ptr = NULL; // Invalidate pointer
  }
}


// ********************* usb debugging *********************
int get_health_pkt(void *dat) {
  COMPILE_TIME_ASSERT(sizeof(struct health_t) <= MAX_RESP_LEN);
  struct health_t * health = (struct health_t*)dat;

  health->uptime_pkt = uptime_cnt;
  health->voltage_pkt = 0;
  health->current_pkt = 0;

  health->ignition_line_pkt = (uint8_t)(current_board->check_ignition());
  health->ignition_can_pkt = (uint8_t)(ignition_can);

  health->controls_allowed_pkt = 1;
  health->gas_interceptor_detected_pkt = 0;
  health->can_rx_errs_pkt = can_rx_errs;
  health->can_send_errs_pkt = can_send_errs;
  health->can_fwd_errs_pkt = can_fwd_errs;
  health->gmlan_send_errs_pkt = 0;
  health->car_harness_status_pkt = car_harness_status;
  health->usb_power_mode_pkt = usb_power_mode;
  health->safety_mode_pkt = (uint8_t)(current_safety_mode);
  health->safety_param_pkt = current_safety_param;
  health->power_save_enabled_pkt = (uint8_t)(0);

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
  if ((usbdata_ep0 == last_usb_data_ptr) & (len_ep0 == sizeof(can_signal_config_t))) {
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
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
  // Clear the buffer after processing for this endpoint
  memset(usbdata, 0, len);
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
        relay_on = 0;
      } else {
        relay_on = 1;
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
      if (setup->b.wLength.w == sizeof(can_signal_config_t) &&
          setup->b.wValue.w < MAX_CONFIG_ENTRIES) {
        config_write_pending = true;
      } else {
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

uint16_t gas_set_0 = 0;
uint16_t gas_set_1 = 0;

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
  // state = FAULT_SCE;
  // can_sce(CAN2);
  llcan_clear_send(CAN2);
}

// CAN 3 read function
void CAN3_RX0_IRQ_Handler(void) {
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN3->sFIFOMailBox[0].RIR >> 21;
    uint64_t can_raw = 0;

    // STEER_ANGLE (MSG_TYPE 1)
    if (steer_angle_major_cfg != NULL && steer_angle_major_cfg->enabled && address == steer_angle_major_cfg->can_id) {
      for (int i = 0; i < steer_angle_major_cfg->msg_len_bytes; i++) {
        can_raw = (can_raw << 8) & 0xFFFFFFFFFFFFFFFF;
        can_raw |= GET_BYTE(&CAN3->sFIFOMailBox[0], i);
      }
      steer_angle = (can_raw >> steer_angle_major_cfg->shift_amt) & ((1 << steer_angle_major_cfg->sig_len) - 1);
      steer_angle = (steer_angle * steer_angle_major_cfg->scale_mult);
      steer_angle += steer_angle_major_cfg->scale_offs;
    }

    // STEER_ANGLE (MSG_TYPE 2)
    if (steer_angle_minor_cfg != NULL && steer_angle_minor_cfg->enabled && address == steer_angle_minor_cfg->can_id) {
      int32_t angle_frac_raw = 0;
      for (int i = 0; i < steer_angle_minor_cfg->msg_len_bytes; i++) {
        can_raw = (can_raw << 8) & 0xFFFFFFFFFFFFFFFF;
        can_raw |= GET_BYTE(&CAN3->sFIFOMailBox[0], i);
      }
      angle_frac_raw = (can_raw >> steer_angle_minor_cfg->shift_amt) & ((1 << steer_angle_minor_cfg->sig_len) - 1);
      angle_frac_raw = (angle_frac_raw * steer_angle_minor_cfg->scale_mult);
      angle_frac_raw += steer_angle_minor_cfg->scale_offs;
      steer_angle += angle_frac_raw;
    }
    
    // VEHICLE_SPEED (MSG_TYPE 3)
    if (vehicle_speed_cfg != NULL && vehicle_speed_cfg->enabled && address == vehicle_speed_cfg->can_id) {
      for (int i = 0; i < steer_angle_minor_cfg->msg_len_bytes; i++) {
        can_raw = (can_raw << 8) & 0xFFFFFFFFFFFFFFFF;
        can_raw |= GET_BYTE(&CAN3->sFIFOMailBox[0], i);
      }
      vss_can = (can_raw >> vehicle_speed_cfg->shift_amt) & ((1 << vehicle_speed_cfg->sig_len) - 1);
      vss_can = (vss_can * vehicle_speed_cfg->scale_mult);
      vss_can += vehicle_speed_cfg->scale_offs;
    }

    can_rx(2);
    CAN3->RF0R |= CAN_RF0R_RFOM0;  // release FIFO
  }
}
void CAN3_SCE_IRQ_Handler(void) {
  // state = FAULT_SCE;
  // can_sce(CAN3);
  llcan_clear_send(CAN3);
}

static void send_can1_message(CAN_FIFOMailBox_TypeDef *to_send, uint8_t *pkt_idx, const char *miss_tag) {
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
    puts(miss_tag);
    puts("CAN1 TSR: ");
    puth(CAN1->TSR);
    puts("\n");
    return;
  }

  *pkt_idx = (*pkt_idx + 1) & COUNTER_CYCLE;
}

// uint8_t gpio = 1;
// timer 3 interrupt. Use this function to perform tasks at specific intervals. see main() for details
uint8_t pkt_idx1 = 0;
uint8_t pkt_idx2= 0;
uint8_t pkt_idx3 = 0;
uint8_t pkt_idx4 = 0;
uint8_t turn = 0;
void TIM3_IRQ_Handler(void) {
  CAN_FIFOMailBox_TypeDef to_send;

  switch (turn) {
    case 0: { // steer_angle
      if (steer_angle_major_cfg->enabled){
        uint8_t dat[6];
        dat[5] = ((state & 0xFU) << 4) | pkt_idx1;
        dat[4] = 0;
        dat[3] = 0;
        dat[2] = (steer_angle >> 8) & 0xFF;
        dat[1] = (steer_angle >> 0) & 0xFF;
        dat[0] = lut_checksum(dat, 6, crc8_lut_1d);

        to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
        to_send.RDHR = dat[4] | (dat[5] << 8);
        to_send.RDTR = 6;
        to_send.RIR = (80 << 21) | 1U;

        send_can1_message(&to_send, &pkt_idx1, "CAN MISS1\n");
      }
      break;
    }

    case 1: { // ENGINE
      uint8_t dat[8];
      
      dat[7] = ((state & 0xFU) << 4) | pkt_idx2;
      dat[6] = (cps_pulse_count >> 0) & 0xFF;
      dat[5] = (cps_dt_us >> 8) & 0xFF;
      dat[4] = (cps_dt_us >> 0) & 0xFF;
      dat[3] = (vss_pulse_count >> 0) & 0xFF;
      dat[2] = (vss_dt_us >> 8) & 0xFF;
      dat[1] = (vss_dt_us >> 0) & 0xFF;
      dat[0] = lut_checksum(dat, 8, crc8_lut_1d);

      to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
      to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
      to_send.RDTR = 8;
      to_send.RIR = (117 << 21) | 1U;

      send_can1_message(&to_send, &pkt_idx2, "CAN MISS2\n");
      break;
    }

    case 2: { // CRUISE
      // TODO: implement this from the ADC readouts
      uint8_t dat[6];
      dat[5] = ((state & 0xFU) << 4) | pkt_idx4;
      dat[4] = 0;
      dat[3] = 0;
      dat[2] = 0;
      dat[1] = 0;
      dat[0] = lut_checksum(dat, 6, crc8_lut_1d);

      to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
      to_send.RDHR = dat[4] | (dat[5] << 8);
      to_send.RDTR = 6;
      to_send.RIR = (256 << 21) | 1U;

      send_can1_message(&to_send, &pkt_idx4, "CAN MISS4\n");
      break;
    }
    case 3: { 
      if(vehicle_speed_cfg->enabled){
        //if we have a CAN VSS source, send it 
        uint8_t dat2[8];
        dat2[5] = ((state & 0xFU) << 4) | pkt_idx3;
        dat2[4] = 0;
        dat2[3] = 0;
        dat2[2] = (vss_can >> 8) & 0xFF;
        dat2[1] = (vss_can >> 0) & 0xFF;
        dat2[0] = lut_checksum(dat2, 6, crc8_lut_1d);

        to_send.RDLR = dat2[0] | (dat2[1] << 8) | (dat2[2] << 16) | (dat2[3] << 24);
        to_send.RDHR = dat2[4] | (dat2[5] << 8);
        to_send.RDTR = 6;
        to_send.RIR = (118 << 21) | 1U;
        send_can1_message(&to_send, &pkt_idx3, "CAN MISS4\n");
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
      if (heartbeat_counter >= 2) {
        // puts("heartbeat not seen for 0x");
        // puth(heartbeat_counter);
        // puts(" seconds. Turning off 12V and IGN.\n");
        if (relay_on != 0 || ignition_on != 0) {
          relay_on = 0;
          ignition_on = 0;
        }
      }
      // check registers
      check_registers();
      uptime_cnt += 1U;
      uptime_cnt &= 0xFFFF;

    }
    loop_counter++;
    loop_counter %= 8U;
  }

  puts(" angle_: ");
  puth(steer_angle);
  puts("\n");
  puts("VSS CNT: ");
  puth(vss_pulse_count);
  puts(" VSS: ");
  puth(vss_dt_us);
  puts(" CPS: ");
  puth(cps_dt_us);
  puts(" VSS_CAN: ");
  puth(vss_can);
  puts("\n");

  TIM9->SR = 0;
}

// ******************************** VSS ********************************
void EXTI4_IRQ_Handler(void) {
  if (EXTI->PR & EXTI_PR_PR4) {
    EXTI->PR = EXTI_PR_PR4;

    vss_pulse_count = (vss_pulse_count + 1) & 0xFFF;

    uint32_t now = TIM2->CNT;
    vss_dt_us = (uint16_t)(now - vss_last_us);  // truncation is okay
    vss_last_us = now;
  }
}
// ******************************** CPS ********************************
void EXTI9_5_IRQ_Handler(void) {
  if (EXTI->PR & EXTI_PR_PR5) {
    EXTI->PR = EXTI_PR_PR5;

    cps_pulse_count = (cps_pulse_count + 1) & 0xFFF;

    uint32_t now = TIM2->CNT;
    cps_dt_us = (uint16_t)(now - cps_last_us);  // truncation is okay
    cps_last_us = now;
  }
}

// ***************************** main code *****************************

// This function is the main application. It is run in a while loop from main() and is interrupted by those specified in main().
void loop(void) {
  // read/write
  set_gpio_output(GPIOB, 0, relay_on);
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

  // ############## FLASH HANDLING #####################
  //config_block_t *cfg = (config_block_t *)CONFIG_FLASH_ADDR; // Pointer to flash content

  uint8_t sig_type_seen[16] = {0};
  bool needs_format = false; 

  config_block_t current_cfg_in_ram;

  memcpy(&current_cfg_in_ram, (void *)CONFIG_FLASH_ADDR, sizeof(config_block_t));

  if (!(current_cfg_in_ram.magic == FLASH_CONFIG_MAGIC &&
        current_cfg_in_ram.crc32 == crc32(&current_cfg_in_ram.entries[0], sizeof(current_cfg_in_ram.entries)))) {
    needs_format = true;
  } else {
    for (int i = 0; i < MAX_CONFIG_ENTRIES; i++) {
      can_signal_config_t *e = &current_cfg_in_ram.entries[i]; 
      if (!e->enabled) continue; 
      if (e->sig_type >= 16) {
        needs_format = true;
        break;
      }
      if (sig_type_seen[e->sig_type]) {
        needs_format = true;
        break; 
      }
      sig_type_seen[e->sig_type] = 1; 
    }
  }

  if (needs_format) {
    flash_unlock();
    flash_config_format();
    flash_lock();
    flash_configured = 0;
    NVIC_SystemReset();
  } else {
    flash_configured = 1;
    for (int i = 0; i < MAX_CONFIG_ENTRIES; i++) {
      can_signal_config_t *e = &current_cfg_in_ram.entries[i];
      if (!e->enabled) continue;

      switch (e->sig_type) {
        case 1: steer_angle_major_cfg = e; puts("steer angle major configured.\n"); break;
        case 2: steer_angle_minor_cfg = e; puts("steer angle minor configured.\n");break;
        case 3: vehicle_speed_cfg = e;puts("vehicle speed configured.\n"); break;
        default: break;
      }
    }
  }

  // ###################################################################

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
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  // Map EXTI4 and EXTI5 to PA4 and PA5
  SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI4 | SYSCFG_EXTICR2_EXTI5);
  // Unmask EXTI lines 4 and 5
  EXTI->IMR |= EXTI_IMR_IM4 | EXTI_IMR_IM5;
  // Trigger on rifallingsing edge
  EXTI->FTSR |= EXTI_FTSR_TR4 | EXTI_FTSR_TR5;
  // Clear pending interrupts
  EXTI->PR = EXTI_PR_PR4 | EXTI_PR_PR5;
  // Enable EXTI interrupts in NVIC
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
  set_gpio_pullup(GPIOA, 6, PULL_UP);

  set_gpio_output(GPIOB, 0, 0); // RELAY off
  set_gpio_output(GPIOB, 1, 1); // IGN_OBDC off

  RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN;
  // watchdog_init();

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
