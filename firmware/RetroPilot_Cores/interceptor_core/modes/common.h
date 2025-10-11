#pragma once

// Mode-specific CAN addresses
#define CAN_DIFFERENTIAL_INPUT  0x300
#define CAN_DIFFERENTIAL_OUTPUT 0x301U
#define CAN_GAS_PEDAL_INPUT     0x200
#define CAN_GAS_PEDAL_OUTPUT    0x201U
#define CAN_UNCONFIGURED_INPUT  0x500U
#define CAN_UNCONFIGURED_OUTPUT 0x501U

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

// Legacy aliases for compatibility
#define pdl0 adc_input_0
#define pdl1 adc_input_1

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

// DAC Safety functions
uint32_t safe_dac_output(uint32_t new_val, uint32_t *last_val, uint8_t channel);
bool validate_dac_peripheral(void);