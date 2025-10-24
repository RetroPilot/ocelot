#pragma once

// Mode-specific CAN addresses
#define CAN_STEER_INPUT     0x12E  // ACTUATOR_STEERING_COMMAND
#define CAN_STEER_OUTPUT    0x12F  // ACTUATOR_STEERING_STATUS
#define CAN_CRUISE_INPUT    0x400  // ACTUATOR_GAS_COMMAND
#define CAN_CRUISE_OUTPUT   0x401  // ACTUATOR_GAS_SENSOR

// Fault state definitions
#define NO_FAULT 0U
#define FAULT_STARTUP 1U
#define FAULT_SENSOR 2U
#define FAULT_SEND 3U
#define FAULT_SCE 4U
#define FAULT_TIMEOUT 5U
#define FAULT_BAD_CHECKSUM 6U
#define FAULT_INVALID 7U
#define FAULT_CONFIG_INVALID 14U
#define FAULT_NOT_CONFIGURED 15U

// External variable declarations
extern uint8_t enabled;
extern uint32_t gas_set;
extern uint8_t state;
extern uint32_t adc[2];
extern uint32_t timeout;
extern uint32_t current_index;
extern unsigned int pkt_idx;
extern uint8_t crc8_lut_1d[256];
extern uint32_t enter_bootloader_mode;

// USB control variables
extern uint8_t motor1_pwm, motor2_pwm;
extern uint8_t motor1_dir, motor2_dir;
extern uint8_t motor1_enable, motor2_enable;
extern uint8_t relay_state;

// Control variables
extern int16_t steer_torque_req;
extern int16_t steer_angle_req;
extern uint8_t steer_mode;
extern uint32_t tps_min, tps_max;
extern uint16_t adc_tol;
extern uint8_t adc_num;

// USB control variables
extern volatile bool usb_ctrl_active;
extern uint32_t ctrl_timeout;

// External declarations for flash config
extern const flash_config_t *signal_configs[];
#define CFG_TYPE_ADC 3
#define CFG_TYPE_SYS 1

// Mode management functions
void setup_mode(uint8_t mode);
uint8_t detect_mode_from_flash(void);

#define COUNTER_CYCLE 0xFU
#define MAX_TIMEOUT 20U
#define USB_CTRL_TIMEOUT 350

// Constants
#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
#define ENTER_SOFTLOADER_MAGIC 0xdeadc0de