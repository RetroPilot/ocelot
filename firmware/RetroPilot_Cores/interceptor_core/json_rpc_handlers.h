#pragma once

#include "drivers/json_rpc.h"

// JSON RPC handler functions
static int json_system_info(const char* params, char* response, int max_len);
static int json_system_methods(const char* params, char* response, int max_len);
static int json_system_reset(const char* params, char* response, int max_len);
static int json_interceptor_get_values(const char* params, char* response, int max_len);
static int json_interceptor_set_dac(const char* params, char* response, int max_len);
static int json_interceptor_emergency_stop(const char* params, char* response, int max_len);

// Implementation
static int json_system_info(const char* params, char* response, int max_len) {
  (void)params;
  char result[256];
  json_build_system_info(result, sizeof(result), "interceptor_core", "\"adc\",\"dac\",\"mode\"");
  return json_build_success(response, max_len, result);
}

static int json_system_methods(const char* params, char* response, int max_len) {
  (void)params;
  return json_build_success(response, max_len,
    "[\"system.info\",\"system.methods\",\"system.reset\",\"interceptor.get_values\",\"test_dac\",\"interceptor.emergency_stop\"]");
}

static int json_system_reset(const char* params, char* response, int max_len) {
  (void)params; (void)response; (void)max_len;
  NVIC_SystemReset();
  return 0;
}

static int json_interceptor_get_values(const char* params, char* response, int max_len) {
  (void)params;
  char result[256];
  char* p = result;
  p += json_strcpy_safe(p, "{", sizeof(result) - (p - result));
  p += json_strcpy_safe(p, "\"adc0\":", sizeof(result) - (p - result));
  p += json_int_to_str(p, adc_input_0, sizeof(result) - (p - result));
  p += json_strcpy_safe(p, ",\"adc1\":", sizeof(result) - (p - result));
  p += json_int_to_str(p, adc_input_1, sizeof(result) - (p - result));
  p += json_strcpy_safe(p, ",\"dac0\":", sizeof(result) - (p - result));
  p += json_int_to_str(p, dac_output_0, sizeof(result) - (p - result));
  p += json_strcpy_safe(p, ",\"dac1\":", sizeof(result) - (p - result));
  p += json_int_to_str(p, dac_output_1, sizeof(result) - (p - result));
  p += json_strcpy_safe(p, ",\"mode\":", sizeof(result) - (p - result));
  p += json_int_to_str(p, current_mode, sizeof(result) - (p - result));
  p += json_strcpy_safe(p, ",\"state\":", sizeof(result) - (p - result));
  p += json_int_to_str(p, state, sizeof(result) - (p - result));
  p += json_strcpy_safe(p, ",\"enabled\":", sizeof(result) - (p - result));
  p += json_strcpy_safe(p, ctrl_enable ? "true" : "false", sizeof(result) - (p - result));
  p += json_strcpy_safe(p, ",\"usb_ctrl\":", sizeof(result) - (p - result));
  p += json_strcpy_safe(p, usb_ctrl_active ? "true" : "false", sizeof(result) - (p - result));
  p += json_strcpy_safe(p, "}", sizeof(result) - (p - result));
  return json_build_success(response, max_len, result);
}

static int json_interceptor_set_dac(const char* params, char* response, int max_len) {
  int dac0 = json_parse_int(params, "dac0");
  int dac1 = json_parse_int(params, "dac1");
  
  if (dac0 >= DAC_MIN_SAFE && dac0 <= DAC_MAX_SAFE) {
    dac_output_0 = dac0;
    usb_ctrl_active = true;
    ctrl_timeout = 0;
    ctrl_enable = true;
    state = NO_FAULT;
  }
  
  if (dac1 >= DAC_MIN_SAFE && dac1 <= DAC_MAX_SAFE) {
    dac_output_1 = dac1;
    usb_ctrl_active = true;
    ctrl_timeout = 0;
    ctrl_enable = true;
    state = NO_FAULT;
  }
  
  char result[64];
  char* p = result;
  p += json_strcpy_safe(p, "{\"dac0\":", sizeof(result) - (p - result));
  p += json_int_to_str(p, dac_output_0, sizeof(result) - (p - result));
  p += json_strcpy_safe(p, ",\"dac1\":", sizeof(result) - (p - result));
  p += json_int_to_str(p, dac_output_1, sizeof(result) - (p - result));
  p += json_strcpy_safe(p, "}", sizeof(result) - (p - result));
  return json_build_success(response, max_len, result);
}

static int json_interceptor_emergency_stop(const char* params, char* response, int max_len) {
  (void)params;
  return json_build_success(response, max_len, "{\"test\":true}");
}

const json_method_t interceptor_methods[] = {
  {"system.info", json_system_info},
  {"system.methods", json_system_methods},
  {"system.reset", json_system_reset},
  {"test_dac", json_interceptor_set_dac},
  {"interceptor.get_values", json_interceptor_get_values},
  {"interceptor.emergency_stop", json_interceptor_emergency_stop},
  {NULL, NULL}
};