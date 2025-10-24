#pragma once
#include "common.h"

static void default_init(void) {
  motor1_pwm = 0;
  motor2_pwm = 0;
  motor1_enable = 0;
  motor2_enable = 0;
  relay_state = 0;
}

static void default_process(void) {
  adc[0] = adc_get(12);
  adc[1] = adc_get(13);
  
  // Individual motor control
  // Motor 1
  set_gpio_output(GPIOB, 10, motor1_enable ? 0 : 1);  // Enable/disable
  set_gpio_output(GPIOB, 0, motor1_dir);               // Direction
  pwm_set(TIM9, 1, motor1_enable ? motor1_pwm : 0);
  
  // Motor 2
  set_gpio_output(GPIOB, 3, motor2_enable ? 0 : 1);   // Enable/disable
  set_gpio_output(GPIOB, 1, motor2_dir);               // Direction
  pwm_set(TIM9, 2, motor2_enable ? motor2_pwm : 0);
  
  // Relay control
  set_gpio_output(GPIOB, 4, relay_state ? 0 : 1);     // Relay (active low)
  
  watchdog_feed();
}

static void default_can_rx_handler(int address, uint8_t *dat) {
  UNUSED(address);
  UNUSED(dat);
  // No CAN handling in default mode - USB control only
}

static void default_timer_handler(void) {
  // Safety timeout counter - runs at 8Hz, so 8 ticks = 1 second
  static uint8_t safety_timeout_counter = 0;
  
  if (usb_ctrl_active) {
    safety_timeout_counter = 0;
  } else {
    safety_timeout_counter++;
    if (safety_timeout_counter >= 8) {
      motor1_enable = 0;
      motor2_enable = 0;
      relay_state = 0;
      safety_timeout_counter = 8; // Cap to prevent overflow
    }
  }
}