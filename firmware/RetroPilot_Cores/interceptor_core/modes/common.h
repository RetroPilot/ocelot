#pragma once

// Mode-specific CAN addresses
#define CAN_DIFFERENTIAL_INPUT  0x300
#define CAN_DIFFERENTIAL_OUTPUT 0x301U
#define CAN_GAS_PEDAL_INPUT     0x200
#define CAN_GAS_PEDAL_OUTPUT    0x201U

// Fault state definitions
#define NO_FAULT 0U
#define FAULT_STARTUP 1U
#define FAULT_SENSOR 2U
#define FAULT_SEND 3U
#define FAULT_SCE 4U
#define FAULT_TIMEOUT 5U
#define FAULT_BAD_CHECKSUM 6U
#define FAULT_INVALID_CKSUM 7U
#define FAULT_REQ_TOO_HIGH 8U
#define FAULT_REQ_INVALID 9U
#define FAULT_ADC_UNCONFIGURED 10U
#define FAULT_TIMEOUT_VSS 11U

// External variable declarations - standardized ADC/DAC names
extern uint32_t adc_input_0, adc_input_1;  // Primary ADC inputs
extern uint32_t dac_output_0, dac_output_1; // Primary DAC outputs



// Mode-specific variables
extern bool override;
extern int32_t magnitude;
extern uint16_t gas_set_0, gas_set_1;
extern int16_t req_mag;
extern bool enable;
extern uint16_t vehicle_speed;
extern uint32_t timeout, timeout_vss;
extern uint32_t current_index, current_index_vss;
extern uint8_t state;
extern unsigned int pkt_idx;
extern int led_value;
extern uint8_t crc8_lut_1d[256];
extern uint32_t enter_bootloader_mode;

// DAC Safety constants
#define DAC_MIN_SAFE 200
#define DAC_MAX_SAFE 3800
#define DAC_SAFE_CENTER 2048
#define MAX_DAC_RATE 50

// DAC Safety variables
extern uint32_t last_dac_0, last_dac_1;

// USB control variables
extern volatile bool usb_ctrl_active;
extern uint32_t ctrl_timeout;

// Control variables used by modes
extern uint32_t ctrl_target_0, ctrl_target_1;
extern int16_t ctrl_magnitude, ctrl_magnitude_alt;
extern bool ctrl_override, ctrl_enable;
extern uint32_t timeout_can, timeout_counter, timeout_vss_counter;
extern uint32_t safety_last_dac_0, safety_last_dac_1;
extern uint8_t current_mode;

// External declarations for flash config
extern const flash_config_t *signal_configs[];
#define CFG_TYPE_ADC 3
#define CFG_TYPE_SYS 1

// Mode management functions
void setup_mode(uint8_t mode);
uint8_t detect_mode_from_flash(void);



// Function implementations
static inline uint32_t safe_dac_output_impl(uint32_t new_val, uint32_t *last_val) {
  if (new_val < DAC_MIN_SAFE) new_val = DAC_MIN_SAFE;
  if (new_val > DAC_MAX_SAFE) new_val = DAC_MAX_SAFE;
  int32_t delta = (int32_t)new_val - (int32_t)*last_val;
  if (delta > MAX_DAC_RATE) {
    new_val = *last_val + MAX_DAC_RATE;
  } else if (delta < -MAX_DAC_RATE) {
    new_val = *last_val - MAX_DAC_RATE;
  }
  *last_val = new_val;
  return new_val;
}

static inline bool validate_dac_peripheral_impl(void) {
  uint32_t dac0_reg = DAC->DHR12R1;
  uint32_t dac1_reg = DAC->DHR12R2;
  if (dac0_reg > 4095 || dac1_reg > 4095) return false;
  if (!(DAC->CR & DAC_CR_EN1) || !(DAC->CR & DAC_CR_EN2)) return false;
  return true;
}