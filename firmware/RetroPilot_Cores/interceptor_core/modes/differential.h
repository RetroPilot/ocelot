#pragma once
#include "common.h"

static void differential_init(void) {
  torque_lut_init();
}

static bool differential_validate_adc(uint32_t adc_val, uint8_t channel) {
  const flash_config_t *cfg = signal_configs[channel + 1]; // ADC configs at indices 1,2
  if (!cfg || cfg->cfg_type != CFG_TYPE_ADC || !(cfg->adc.adc_en & 1)) return false; // Fault if unconfigured
  
  uint32_t center = cfg->adc.adc1;     // Expected center position
  uint16_t tolerance = cfg->adc.adc_tolerance; // +/- deviation allowed
  
  return (ABS((int32_t)adc_val - (int32_t)center) <= tolerance);
}

static void differential_debug_output(bool relay_state) {
  puts("ADC0:"); puth(adc_input_0);
  puts(" ADC1:"); puth(adc_input_1);
  puts(" DAC0:"); puth(dac_output_0);
  puts(" DAC1:"); puth(dac_output_1);
  puts(" Relay:"); puth(relay_state);
  puts(" State:"); puth(state);
  puts(" Mag:"); puth(ctrl_magnitude_alt);
  puts(" Ovr:"); puth(ctrl_override);
  puts(" TScale:"); puth(get_torque_scale_percent(vehicle_speed));
  puts(" ReqMag:"); puth(ctrl_magnitude);
  puts(" CtrlTarget0:"); puth(ctrl_target_0);
  puts(" CtrlTarget1:"); puth(ctrl_target_1);
  puts(" Enable:"); puth(ctrl_enable);
  puts("\n");
}

static void differential_process(void) {
  // Read ADC
  adc_input_0 = adc_get(12);
  adc_input_1 = adc_get(13);

  ctrl_magnitude_alt = ABS((int32_t) adc_input_0 - (int32_t) adc_input_1);
  
  // Get configurable override threshold from SYS config
  uint16_t override_threshold = 0x150; // Default fallback
  const flash_config_t *sys_cfg = signal_configs[0];
  if (sys_cfg && sys_cfg->cfg_type == CFG_TYPE_SYS) {
    override_threshold = (sys_cfg->sys.cfg_extra[1] << 8) | sys_cfg->sys.cfg_extra[2];
    if (override_threshold == 0) override_threshold = 0x150; // Use default if not set
  }
  
  ctrl_override = ABS(ctrl_magnitude_alt) > override_threshold;

  ctrl_magnitude = (ctrl_target_0 - ctrl_target_1);
  
  // Limit ctrl_magnitude first
  if (ABS(ctrl_magnitude) > 400){
    ctrl_magnitude = 0;
  }
  
  uint16_t scale_percent = get_torque_scale_percent(vehicle_speed);
  // Ensure minimum 50% scaling for testing at low speeds
  if (scale_percent < 50) scale_percent = 50;
  int16_t scaled_torque = (ctrl_magnitude * scale_percent) / 100; 

  // Differential torque logic
  if ((state == NO_FAULT) && ctrl_enable) {
    // Get center points from flash config
    uint32_t center_0 = DAC_SAFE_CENTER; // Default fallback
    uint32_t center_1 = DAC_SAFE_CENTER; // Default fallback
    
    const flash_config_t *cfg0 = signal_configs[1]; // ADC config for channel 0
    const flash_config_t *cfg1 = signal_configs[2]; // ADC config for channel 1
    
    if (cfg0 && cfg0->cfg_type == CFG_TYPE_ADC) {
      center_0 = cfg0->adc.adc1; // Center point from flash
    }
    if (cfg1 && cfg1->cfg_type == CFG_TYPE_ADC) {
      center_1 = cfg1->adc.adc1; // Center point from flash
    }
    
    int32_t raw_dac_0 = (int32_t)center_0 - scaled_torque;
    int32_t raw_dac_1 = (int32_t)center_1 + scaled_torque;
    // Clamp to 12-bit DAC range only
    dac_output_0 = (raw_dac_0 < 0) ? 0 : ((raw_dac_0 > 4095) ? 4095 : raw_dac_0);
    dac_output_1 = (raw_dac_1 < 0) ? 0 : ((raw_dac_1 > 4095) ? 4095 : raw_dac_1);
  } else {
    // In fault state, use safe center values
    dac_output_0 = DAC_SAFE_CENTER;
    dac_output_1 = DAC_SAFE_CENTER;
  }
  
  dac_set(0, dac_output_0);
  dac_set(1, dac_output_1);
  
  watchdog_feed();
}

static void differential_can_rx_handler(int address, uint8_t *dat) {
  if (address == CAN_DIFFERENTIAL_INPUT) {
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

    // Normal packet processing
    uint16_t value_0 = (dat[2] << 8) | dat[1];
    uint16_t value_1 = (dat[4] << 8) | dat[3];
    ctrl_enable = ((dat[5] >> 7) & 1U) != 0U;
    uint8_t index = dat[5] & 0xFU;
    
    if (dat[0] == lut_checksum(dat, 6, crc8_lut_1d)) {
      if (((timeout_counter + 1U) & 0xFU) == index) {
        if (ctrl_enable) {
          ctrl_target_0 = value_0;
          ctrl_target_1 = value_1;
        } else {
          if ((value_0 == 0U) && (value_1 == 0U)) {
            state = NO_FAULT;
          } else {
            state = FAULT_INVALID_CKSUM;
          }
          ctrl_target_0 = 0;
          ctrl_target_1 = 0;
          ctrl_magnitude = 0;
        }
        timeout_can = 0;
      }
      timeout_counter = index;
    } else {
      state = FAULT_BAD_CHECKSUM;
    }
  }
  
  if (address == 0x76) {
    // Vehicle speed processing
    uint16_t value_1 = (dat[6] << 8) | dat[5];
    uint8_t index = dat[7] & 0xFU;
    if (dat[0] == lut_checksum(dat, 8, crc8_lut_1d)) {
      if (((timeout_vss_counter + 1U) & 0xFU) == index) {
        vehicle_speed = value_1;
        timeout_vss = 0;
      }
      timeout_vss_counter = index;
    }
  }
}

static void differential_timer_handler(void) {
  // Send differential output
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    dat[1] = (adc_input_0 >> 0) & 0xFFU;
    dat[2] = (adc_input_0 >> 8) & 0xFFU;
    dat[3] = (adc_input_1 >> 0) & 0xFFU;
    dat[4] = (adc_input_1 >> 8) & 0xFFU;
    dat[5] = ctrl_override;
    dat[6] = 0;
    dat[7] = ((state & 0xFU) << 4) | pkt_idx;
    dat[0] = lut_checksum(dat, 6, crc8_lut_1d);
    
    CAN1->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN1->sTxMailBox[0].TDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
    CAN1->sTxMailBox[0].TDTR = 8;
    CAN1->sTxMailBox[0].TIR = (CAN_DIFFERENTIAL_OUTPUT << 21) | 1U;
    ++pkt_idx;
    pkt_idx &= 0xFU;
  } else {
    state = FAULT_SEND;
  }

  // Blink LED
  current_board->set_led(LED_GREEN, led_value);
  led_value = !led_value;

  // Timeout handling
  if (timeout_can == 700U) {
    state = FAULT_TIMEOUT;
    ctrl_enable = 0;
  } else {
    timeout_can += 1U;
  }

  if (timeout_vss == 300U) {
    state = FAULT_TIMEOUT_VSS;
    vehicle_speed = 0;
    ctrl_enable = 0;
  } else {
    timeout_vss += 1U;
  }
}