#pragma once
#include "common.h"

static void gas_pedal_init(void) {
  // Gas pedal mode initialization
}

static bool gas_pedal_validate_adc(uint32_t adc_val, uint8_t channel) {
  const flash_config_t *cfg = signal_configs[channel + 1]; // ADC configs at indices 1,2
  if (!cfg || cfg->cfg_type != CFG_TYPE_ADC || !(cfg->adc.adc_en & 1)) return false; // Fault if unconfigured
  
  uint32_t min_val = cfg->adc.adc2;    // Minimum valid (e.g., 200 = 0.2V)
  uint32_t max_val = cfg->adc.adc1;    // Maximum valid (e.g., 4000 = 4.0V)
  
  return (adc_val >= min_val && adc_val <= max_val);
}

static void gas_pedal_debug_output(bool relay_state) {
  puts("ADC0:"); puth(adc_input_0);
  puts(" ADC1:"); puth(adc_input_1);
  puts(" DAC0:"); puth(dac_output_0);
  puts(" DAC1:"); puth(dac_output_1);
  puts(" Relay:"); puth(relay_state);
  puts(" State:"); puth(state);
  puts(" GasSet0:"); puth(gas_set_0);
  puts(" GasSet1:"); puth(gas_set_1);
  puts(" Enable:"); puth(enable);
  puts("\n");
}

static void gas_pedal_process(void) {
  // Read ADC
  adc_input_0 = adc_get(12);
  adc_input_1 = adc_get(13);

  // Gas pedal logic: MAX of set value and ADC
  if (state == NO_FAULT) {
    uint32_t raw_dac_0 = MAX(gas_set_0, adc_input_0);
    uint32_t raw_dac_1 = MAX(gas_set_1, adc_input_1);
    dac_output_0 = safe_dac_output(raw_dac_0, &last_dac_0, 0);
    dac_output_1 = safe_dac_output(raw_dac_1, &last_dac_1, 1);
  } else {
    // In fault state, use safe center values
    dac_output_0 = safe_dac_output(DAC_SAFE_CENTER, &last_dac_0, 0);
    dac_output_1 = safe_dac_output(DAC_SAFE_CENTER, &last_dac_1, 1);
  }
  
  dac_set(0, dac_output_0);
  dac_set(1, dac_output_1);

  watchdog_feed();
}

static void gas_pedal_can_rx_handler(int address, uint8_t *dat) {
  if (address == CAN_GAS_PEDAL_INPUT) {
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
    bool enable_bit = ((dat[5] >> 7) & 1U) != 0U;
    uint8_t index = dat[5] & 0xFU;
    
    if (dat[0] == lut_checksum(dat, 6, crc8_lut_1d)) {
      if (((current_index + 1U) & 0xFU) == index) {
        enable = enable_bit;  // Set global enable variable
        if (enable) {
          gas_set_0 = value_0;
          gas_set_1 = value_1;
        } else {
          if ((value_0 == 0U) && (value_1 == 0U)) {
            state = NO_FAULT;
          } else {
            state = FAULT_INVALID_CKSUM;
          }
          gas_set_0 = 0;
          gas_set_1 = 0;
        }
        timeout = 0;
      }
      current_index = index;
    } else {
      state = FAULT_BAD_CHECKSUM;
    }
  }
}

static void gas_pedal_timer_handler(void) {
  // Send gas pedal output
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[6];
    dat[1] = (adc_input_0 >> 0) & 0xFFU;
    dat[2] = (adc_input_0 >> 8) & 0xFFU;
    dat[3] = (adc_input_1 >> 0) & 0xFFU;
    dat[4] = (adc_input_1 >> 8) & 0xFFU;
    dat[5] = ((state & 0xFU) << 4) | pkt_idx;
    dat[0] = lut_checksum(dat, 6, crc8_lut_1d);
    
    CAN1->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN1->sTxMailBox[0].TDHR = dat[4] | (dat[5] << 8);
    CAN1->sTxMailBox[0].TDTR = 6;
    CAN1->sTxMailBox[0].TIR = (CAN_GAS_PEDAL_OUTPUT << 21) | 1U;
    ++pkt_idx;
    pkt_idx &= 0xFU;
  } else {
    state = FAULT_SEND;
  }

  // Blink LED
  current_board->set_led(LED_GREEN, led_value);
  led_value = !led_value;

  // Timeout handling
  if (timeout == 700) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }
}