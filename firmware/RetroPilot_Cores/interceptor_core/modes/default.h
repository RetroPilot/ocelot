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
  adc_input_0 = adc_get(12);
  adc_input_1 = adc_get(13);
  
  if (!usb_ctrl_active) {
    dac_output_0 = safe_dac_output_impl(adc_input_0, &safety_last_dac_0);
    dac_output_1 = safe_dac_output_impl(adc_input_1, &safety_last_dac_1);
    dac_set(0, dac_output_0);
    dac_set(1, dac_output_1);
  } else {
    // When USB control is active, apply safety limits and set DAC
    uint32_t safe_dac0 = safe_dac_output_impl(dac_output_0, &safety_last_dac_0);
    uint32_t safe_dac1 = safe_dac_output_impl(dac_output_1, &safety_last_dac_1);
    dac_set(0, safe_dac0);
    dac_set(1, safe_dac1);
  }
  
  watchdog_feed();
}

static void default_can_rx_handler(int address, uint8_t *dat) {
  UNUSED(address);
  UNUSED(dat);
  // No CAN processing in unconfigured mode
}

static void default_timer_handler(void) {
  // Blink LED only - no CAN output in unconfigured mode
  current_board->set_led(LED_GREEN, led_value);
  led_value = !led_value;
}