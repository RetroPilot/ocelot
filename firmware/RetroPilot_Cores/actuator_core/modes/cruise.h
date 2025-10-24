#pragma once
#include "common.h"

static void cruise_init(void) {
  gas_set = 0;
  enabled = 0;
}

static void cruise_process(void) {
  adc[0] = adc_get(12);
  adc[1] = adc_get(13);
  
  // Position-based throttle control (from commented motor control logic)
  if (enabled && gas_set > 0) {
    if (gas_set > (tps_max - tps_min)) gas_set = tps_max - tps_min;

    uint32_t target_adc = gas_set + tps_min;
    uint32_t lower_deadzone = (target_adc > adc_tol) ? target_adc - adc_tol : 0;
    uint32_t upper_deadzone = target_adc + adc_tol;

    if (adc[adc_num] >= lower_deadzone && adc[adc_num] <= upper_deadzone) {
      // In deadzone - hold position
      set_gpio_output(GPIOB, 10, 1);  // Disable motor1
      pwm_set(TIM9, 1, 0);
    } else if (adc[adc_num] < target_adc) {
      // Need to increase throttle
      set_gpio_output(GPIOB, 10, 0);  // Enable motor1
      set_gpio_output(GPIOB, 0, 1);   // Forward direction
      pwm_set(TIM9, 1, 50);           // 50% PWM
    } else {
      // Need to decrease throttle
      set_gpio_output(GPIOB, 10, 0);  // Enable motor1
      set_gpio_output(GPIOB, 0, 0);   // Reverse direction
      pwm_set(TIM9, 1, 50);           // 50% PWM
    }

    // Engage clutch (Motor2)
    set_gpio_output(GPIOB, 3, 0);   // Enable motor2
    pwm_set(TIM9, 2, 25);           // Light clutch engagement
  } else {
    // Disable everything
    set_gpio_output(GPIOB, 3, 1);   // Disable motor2 (clutch)
    set_gpio_output(GPIOB, 10, 1);  // Disable motor1
    set_gpio_output(GPIOB, 0, 1);   // Default direction
    pwm_set(TIM9, 1, 0);
    pwm_set(TIM9, 2, 0);
    gas_set = 0;
  }
  
  watchdog_feed();
}

static void cruise_can_rx_handler(int address, uint8_t *dat) {
  if (address == CAN_CRUISE_INPUT) {
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

    // Parse ACTUATOR_GAS_COMMAND (existing logic)
    uint16_t value_0 = (dat[2] << 8) | dat[1];  // THROTTLE_REQ
    bool enable = ((dat[5] >> 7) & 1U) != 0U;   // ENABLE
    uint8_t index = dat[5] & COUNTER_CYCLE;     // COUNTER
    
    if (dat[0] == lut_checksum(dat, 6, crc8_lut_1d)) {
      if (((current_index + 1U) & COUNTER_CYCLE) == index) {
        if (enable) {
          enabled = 1;
          if (value_0 < 2000) {
            gas_set = value_0;
          } else {
            state = FAULT_INVALID;
          }
        } else {
          enabled = 0;
          gas_set = 0;
          if (value_0 == 0U) {
            state = NO_FAULT;
          } else {
            state = FAULT_INVALID;
          }
        }
        timeout = 0;
      }
      current_index = index;
    } else {
      state = FAULT_BAD_CHECKSUM;
    }
  }
}

static void cruise_timer_handler(void) {
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    // ACTUATOR_GAS_SENSOR format (existing logic)
    dat[1] = (adc[0] >> 0) & 0xFFU;        // THROTTLE_POS low
    dat[2] = (adc[0] >> 8) & 0xFFU;        // THROTTLE_POS high
    dat[3] = (adc[1] >> 0) & 0xFFU;        // VSS low
    dat[4] = (adc[1] >> 8) & 0xFFU;        // VSS high
    dat[5] = (0 >> 0) & 0xFFU;             // Reserved
    dat[6] = (0 >> 8) & 0xFFU;             // Reserved
    dat[7] = ((state & 0xFU) << 4) | (pkt_idx & 0xF);  // STATE | COUNTER
    dat[0] = lut_checksum(dat, 8, crc8_lut_1d);        // CHECKSUM
    
    CAN_FIFOMailBox_TypeDef to_send;
    to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
    to_send.RDTR = 8;
    to_send.RIR = (CAN_CRUISE_OUTPUT << 21) | 1U;
    can_send(&to_send, 0, false);
    llcan_clear_send(CAN1);
    ++pkt_idx;
    pkt_idx &= COUNTER_CYCLE;
  } else {
    state = FAULT_SEND;
  }
}