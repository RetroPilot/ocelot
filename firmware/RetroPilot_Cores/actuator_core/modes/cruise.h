#pragma once
#include "common.h"

// Cruise-specific variables
static uint32_t gas_set;
static uint32_t tps_min, tps_max;
static uint16_t adc_tol;
static uint8_t adc_num;
static uint8_t motor_bridge, motor_polarity;
static uint8_t clutch_bridge, clutch_polarity;
static bool motor_configured, clutch_configured;

static void cruise_init(void) {
  gas_set = 0;
  enabled = 0;
  
  // Load ADC config
  if (signal_configs[1] != NULL && signal_configs[1]->cfg_type == CFG_TYPE_ADC) {
    tps_min = signal_configs[1]->adc.adc1;
    tps_max = signal_configs[1]->adc.adc2; 
    adc_tol = signal_configs[1]->adc.adc_tolerance;
    adc_num = signal_configs[1]->adc.adc_num;
  } else {
    puts("ADC not configured. Please use the config tool to set.\n");
    state = FAULT_NOT_CONFIGURED;
    return;
  }
  
  // Find motor and clutch configs
  motor_configured = false;
  clutch_configured = false;
  
  for (int i = 2; i < MAX_CONFIG_ENTRIES; i++) {
    if (signal_configs[i] && signal_configs[i]->cfg_type == CFG_TYPE_MOTOR) {
      if (signal_configs[i]->motor.type == 1) {  // motor
        motor_bridge = signal_configs[i]->motor.bridge_channel;
        motor_polarity = signal_configs[i]->motor.polarity;
        motor_configured = true;
      } else if (signal_configs[i]->motor.type == 2) {  // clutch
        clutch_bridge = signal_configs[i]->motor.bridge_channel;
        clutch_polarity = signal_configs[i]->motor.polarity;
        clutch_configured = true;
      }
    }
  }
  
  if (!motor_configured || !clutch_configured) {
    puts("Motor configs not found. Please configure motor and clutch.\n");
    state = FAULT_NOT_CONFIGURED;
  }
}

static void cruise_process(void) {
  adc[0] = adc_get(12);
  adc[1] = adc_get(13);
  
  // check for clutch failure. is this necessary?
  if (!enabled && adc[adc_num] > (tps_min + adc_tol * 2)) {
    state = FAULT_SENSOR;
  }
  
  // position throttle control
  if (enabled && gas_set > 0) {
    if (gas_set > (tps_max - tps_min)) gas_set = tps_max - tps_min;

    uint32_t target_adc = gas_set + tps_min;
    uint32_t lower_deadzone = (target_adc > adc_tol) ? target_adc - adc_tol : 0;
    uint32_t upper_deadzone = target_adc + adc_tol;

    if (adc[adc_num] >= lower_deadzone && adc[adc_num] <= upper_deadzone) {
      // hold position
      set_gpio_output(GPIOB, 10, 1);
      pwm_set(TIM9, motor_bridge, 0);
    } else if (adc[adc_num] < target_adc && adc[adc_num] < tps_max) {
      // increase if not at the endstops
      set_gpio_output(GPIOB, 10, 0); 
      set_gpio_output(GPIOB, 0, (motor_polarity == 2) ? 0 : 1);
      pwm_set(TIM9, motor_bridge, 100);
    } else if (adc[adc_num] > target_adc) {
      // decrease
      set_gpio_output(GPIOB, 10, 0);
      set_gpio_output(GPIOB, 0, (motor_polarity == 2) ? 1 : 0); 
      pwm_set(TIM9, motor_bridge, 100); 
    }
    set_gpio_output(GPIOB, 3, 0); 
    pwm_set(TIM9, clutch_bridge, 100);
  } else {
    set_gpio_output(GPIOB, 3, 1); 
    set_gpio_output(GPIOB, 10, 1);
    if (motor_configured) pwm_set(TIM9, motor_bridge, 0);
    if (clutch_configured) pwm_set(TIM9, clutch_bridge, 0);
    gas_set = 0;
  }
  
  watchdog_feed();
}

static void cruise_can_rx_handler(int address, uint8_t *dat) {
  if (address == CAN_CRUISE_INPUT) {
    // bootloader entry
    if (GET_BYTES_04(&CAN1->sFIFOMailBox[0]) == 0xdeadface) {
      if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x0ab00b1e) {
        enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
        NVIC_SystemReset();
      } else if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x02b00b1e) {
        enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
        NVIC_SystemReset();
      }
    }
    uint16_t value_0 = (dat[2] << 8) | dat[1];
    bool enable = ((dat[5] >> 7) & 1U) != 0U;
    uint8_t index = dat[5] & COUNTER_CYCLE;
    
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
    dat[1] = (adc[0] >> 0) & 0xFFU; 
    dat[2] = (adc[0] >> 8) & 0xFFU;
    dat[3] = (adc[1] >> 0) & 0xFFU;
    dat[4] = (adc[1] >> 8) & 0xFFU;
    dat[5] = (0 >> 0) & 0xFFU;
    dat[6] = (0 >> 8) & 0xFFU;
    dat[7] = ((state & 0xFU) << 4) | (pkt_idx & 0xF);
    dat[0] = lut_checksum(dat, 8, crc8_lut_1d);
    
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