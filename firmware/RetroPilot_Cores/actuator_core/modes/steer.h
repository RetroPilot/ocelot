#pragma once
#include "common.h"

// Steer-specific variables
static int16_t steer_torque_req;
static int16_t steer_angle_req;
static uint8_t steer_mode;

// placeholder logic just to test this mode of Actuator Core
// TODO: actually write something useable

static void steer_init(void) {
  steer_torque_req = 0;
  steer_angle_req = 0;
  steer_mode = 0;
}

static void steer_process(void) {
  adc[0] = adc_get(12);
  adc[1] = adc_get(13);
  
  // Convert torque request to motor speeds
  // Positive torque = turn right, negative = turn left
  int16_t motor1_speed = 0;  // Left motor
  int16_t motor2_speed = 0;  // Right motor
  
  if (enabled && (steer_torque_req != 0 || steer_angle_req != 0)) {
    if (steer_mode == 0) {  // Torque mode
      // Simple differential: one motor forward, one reverse
      if (steer_torque_req > 0) {  // Turn right
        motor1_speed = steer_torque_req / 10;   // Scale down
        motor2_speed = -steer_torque_req / 10;
      } else {  // Turn left
        motor1_speed = steer_torque_req / 10;
        motor2_speed = -steer_torque_req / 10;
      }
    } else {  // Angle mode (future implementation)
      motor1_speed = 0;
      motor2_speed = 0;
    }
    
    // Limit speeds to 0-100%
    if (motor1_speed > 100) motor1_speed = 100;
    if (motor1_speed < -100) motor1_speed = -100;
    if (motor2_speed > 100) motor2_speed = 100;
    if (motor2_speed < -100) motor2_speed = -100;
    
    // Set motor directions and PWM
    set_gpio_output(GPIOB, 10, 0);  // Enable motor1
    set_gpio_output(GPIOB, 3, 0);   // Enable motor2
    set_gpio_output(GPIOB, 0, motor1_speed >= 0 ? 1 : 0);  // Motor1 direction
    set_gpio_output(GPIOB, 1, motor2_speed >= 0 ? 1 : 0);  // Motor2 direction
    
    pwm_set(TIM9, 1, motor1_speed >= 0 ? motor1_speed : -motor1_speed);
    pwm_set(TIM9, 2, motor2_speed >= 0 ? motor2_speed : -motor2_speed);
  } else {
    // Disable motors
    set_gpio_output(GPIOB, 10, 1);  // Disable motor1
    set_gpio_output(GPIOB, 3, 1);   // Disable motor2
    pwm_set(TIM9, 1, 0);
    pwm_set(TIM9, 2, 0);
  }
  
  watchdog_feed();
}

static void steer_can_rx_handler(int address, uint8_t *dat) {
  if (address == CAN_STEER_INPUT) {
    // Bootloader entry
    if (GET_BYTES_04(&CAN1->sFIFOMailBox[0]) == 0xdeadface) {
      if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x0ab00b1e) {
        enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
        NVIC_SystemReset();
      } else if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x02b00b1e) {
        enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
        NVIC_SystemReset();
      }
    }
    
    // Parse ACTUATOR_STEERING_COMMAND
    steer_torque_req = (int16_t)((dat[5] << 8) | dat[4]);  // REQUESTED_STEER_TORQUE
    steer_angle_req = (int16_t)((dat[3] << 8) | dat[2]);   // REQUESTED_STEER_ANGLE
    steer_mode = (dat[1] >> 4) & 0x3;                      // STEER_MODE
    uint8_t counter = dat[1] & 0xF;                        // COUNTER
    uint8_t checksum = dat[0];                             // CHECKSUM
    
    if (checksum == lut_checksum(dat, 6, crc8_lut_1d)) {
      if (((current_index + 1U) & COUNTER_CYCLE) == counter) {
        enabled = (steer_torque_req != 0 || steer_angle_req != 0) ? 1 : 0;
        timeout = 0;
        state = NO_FAULT;
      }
      current_index = counter;
    } else {
      state = FAULT_BAD_CHECKSUM;
    }
  }
}

static void steer_timer_handler(void) {
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[7];
    // ACTUATOR_STEERING_STATUS format
    dat[1] = (adc[0] >> 0) & 0xFFU;        // STEERING_TORQUE_DRIVER low
    dat[2] = (adc[0] >> 8) & 0xFFU;        // STEERING_TORQUE_DRIVER high
    dat[3] = (adc[1] >> 0) & 0xFFU;        // STEERING_TORQUE_EPS low
    dat[4] = (adc[1] >> 8) & 0xFFU;        // STEERING_TORQUE_EPS high
    dat[5] = (enabled ? 1 : 0) | ((state & 0xF) << 4);  // STEERING_OK | STATUS
    dat[6] = pkt_idx & 0xF;                // COUNTER
    dat[0] = lut_checksum(dat, 7, crc8_lut_1d);  // CHECKSUM
    
    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16);
    to_send.RDTR = 7;
    to_send.RIR = (CAN_STEER_OUTPUT << 21) | 1U;
    can_send(&to_send, 0, false);
    llcan_clear_send(CAN1);
    ++pkt_idx;
    pkt_idx &= COUNTER_CYCLE;
  } else {
    state = FAULT_SEND;
  }
}