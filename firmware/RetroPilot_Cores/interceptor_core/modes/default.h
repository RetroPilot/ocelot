#pragma once
#include "common.h"

static void default_init(void) {
  // Safe defaults - relay OFF, no processing
}

static bool default_validate_adc(uint32_t adc_val, uint8_t channel) {
  UNUSED(adc_val);
  UNUSED(channel);
  return true; // No validation in unconfigured mode
}

static void default_debug_output(bool relay_state) {
  puts("ADC0:"); puth(adc_input_0);
  puts(" ADC1:"); puth(adc_input_1);
  puts(" DAC0:"); puth(dac_output_0);
  puts(" DAC1:"); puth(dac_output_1);
  puts(" Relay:"); puth(relay_state);
  puts(" State:"); puth(state);
  puts("\n");
}

static void default_process(void) {
  // Pass-through: DAC = ADC (with safety limits)
  adc_input_0 = adc_get(12);
  adc_input_1 = adc_get(13);
  
  dac_output_0 = safe_dac_output(adc_input_0, &last_dac_0, 0);
  dac_output_1 = safe_dac_output(adc_input_1, &last_dac_1, 1);
  dac_set(0, dac_output_0);
  dac_set(1, dac_output_1);
  
  // Relay controlled by universal state check
  
  watchdog_feed();
}

static void default_can_rx_handler(int address, uint8_t *dat) {
  UNUSED(dat);
  // Only handle bootloader commands
  if (address == CAN_UNCONFIGURED_INPUT) {
    if (GET_BYTES_04(&CAN1->sFIFOMailBox[0]) == 0xdeadface) {
      if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x0ab00b1e) {
        enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
        NVIC_SystemReset();
      } else if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x02b00b1e) {
        enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
        NVIC_SystemReset();
      }
    }
  }
}

static void default_timer_handler(void) {
  // Send unconfigured status message
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8] = {0};
    dat[0] = 0xDE; // Unconfigured marker
    dat[1] = 0xAD;
    dat[7] = pkt_idx;
    
    CAN1->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN1->sTxMailBox[0].TDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
    CAN1->sTxMailBox[0].TDTR = 8;
    CAN1->sTxMailBox[0].TIR = (CAN_UNCONFIGURED_OUTPUT << 21) | 1U;
    ++pkt_idx;
    pkt_idx &= 0xFU;
  }
  
  // Blink LED
  current_board->set_led(LED_GREEN, led_value);
  led_value = !led_value;
}