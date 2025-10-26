#pragma once

#include "drivers/json_rpc.h"

extern uint8_t current_mode;
extern uint8_t motor1_pwm, motor2_pwm;
extern uint8_t motor1_dir, motor2_dir;
extern uint8_t motor1_enable, motor2_enable;
extern uint8_t relay_state;
extern uint32_t adc[2];
extern uint8_t state;
extern volatile bool usb_ctrl_active;
extern uint32_t ctrl_timeout;
extern bool ctrl_enable;



static int json_system_info(const char* params, char* response, int max_len) {
  (void)params;
  char result[256];
  json_build_system_info(result, sizeof(result), "actuator_core", "\"motor1\",\"motor2\",\"relay\",\"adc\"");
  return json_build_success(response, max_len, result);
}

static int json_system_methods(const char* params, char* response, int max_len) {
  (void)params;
  return json_build_success(response, max_len,
    "[\"system.info\",\"system.methods\",\"system.reset\",\"actuator.get_status\",\"actuator.motor\",\"actuator.relay\"]");
}

static int json_system_reset(const char* params, char* response, int max_len) {
  (void)params; (void)response; (void)max_len;
  NVIC_SystemReset();
  return 0;
}

static int json_actuator_get_status(const char* params, char* response, int max_len) {
  (void)params;
  char result[256];
  char* p = result;
  int remaining = sizeof(result);
  
  p += json_strcpy_safe(p, "{", remaining);
  remaining -= (p - result);
  
  // Build JSON fields more efficiently
  char temp[32];
  json_int_to_str(temp, motor1_pwm, sizeof(temp));
  p += json_strcpy_safe(p, "\"motor1_pwm\":", remaining); p += json_strcpy_safe(p, temp, remaining);
  json_int_to_str(temp, motor2_pwm, sizeof(temp));
  p += json_strcpy_safe(p, ",\"motor2_pwm\":", remaining); p += json_strcpy_safe(p, temp, remaining);
  json_int_to_str(temp, motor1_dir, sizeof(temp));
  p += json_strcpy_safe(p, ",\"motor1_dir\":", remaining); p += json_strcpy_safe(p, temp, remaining);
  json_int_to_str(temp, motor2_dir, sizeof(temp));
  p += json_strcpy_safe(p, ",\"motor2_dir\":", remaining); p += json_strcpy_safe(p, temp, remaining);
  p += json_strcpy_safe(p, ",\"motor1_enable\":", remaining); p += json_strcpy_safe(p, motor1_enable ? "true" : "false", remaining);
  p += json_strcpy_safe(p, ",\"motor2_enable\":", remaining); p += json_strcpy_safe(p, motor2_enable ? "true" : "false", remaining);
  p += json_strcpy_safe(p, ",\"relay_state\":", remaining); p += json_strcpy_safe(p, relay_state ? "true" : "false", remaining);
  json_int_to_str(temp, adc[0], sizeof(temp));
  p += json_strcpy_safe(p, ",\"adc0\":", remaining); p += json_strcpy_safe(p, temp, remaining);
  json_int_to_str(temp, adc[1], sizeof(temp));
  p += json_strcpy_safe(p, ",\"adc1\":", remaining); p += json_strcpy_safe(p, temp, remaining);
  json_int_to_str(temp, state, sizeof(temp));
  p += json_strcpy_safe(p, ",\"state\":", remaining); p += json_strcpy_safe(p, temp, remaining);
  json_int_to_str(temp, current_mode, sizeof(temp));
  p += json_strcpy_safe(p, ",\"mode\":", remaining); p += json_strcpy_safe(p, temp, remaining);
  p += json_strcpy_safe(p, ",\"usb_ctrl\":", remaining); p += json_strcpy_safe(p, usb_ctrl_active ? "true" : "false", remaining);
  p += json_strcpy_safe(p, "}", remaining);
  
  return json_build_success(response, max_len, result);
}

static int json_actuator_motor(const char* params, char* response, int max_len) {
  int motor_num = json_parse_int(params, "m");
  int pwm = json_parse_int(params, "p");
  int dir = json_parse_int(params, "d");
  
  if (motor_num == 1 && pwm >= 0 && pwm <= 100 && dir >= 0 && dir <= 1) {
    motor1_pwm = pwm;
    motor1_dir = dir;
    motor1_enable = (pwm > 0) ? 1 : 0;
    usb_ctrl_active = true;
    ctrl_timeout = 0;
  } else if (motor_num == 2 && pwm >= 0 && pwm <= 100 && dir >= 0 && dir <= 1) {
    motor2_pwm = pwm;
    motor2_dir = dir;
    motor2_enable = (pwm > 0) ? 1 : 0;
    usb_ctrl_active = true;
    ctrl_timeout = 0;
  } else {
    return json_build_error(response, max_len, "invalid_params", "Motor 1-2, PWM 0-100, direction 0-1");
  }
  
  return json_build_success(response, max_len, "\"ok\"");
}

static int json_actuator_relay(const char* params, char* response, int max_len) {
  int value = json_parse_int(params, "value");
  
  if (value >= 0 && value <= 1) {
    relay_state = value;
    usb_ctrl_active = true;
    ctrl_timeout = 0;
    return json_build_success(response, max_len, "\"ok\"");
  }
  
  return json_build_error(response, max_len, "invalid_params", "Value must be 0 or 1");
}

const json_method_t actuator_methods[] = {
  {"system.info", json_system_info},
  {"system.methods", json_system_methods},
  {"system.reset", json_system_reset},
  {"actuator.get_status", json_actuator_get_status},
  {"actuator.motor", json_actuator_motor},
  {"actuator.relay", json_actuator_relay},
  {NULL, NULL}
};