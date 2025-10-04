#ifndef JSON_RPC_H
#define JSON_RPC_H

#include <stdint.h>
#include <stdbool.h>
#include "obj/gitversion.h"

#define JSON_MAX_RESPONSE_SIZE 2048
#define JSON_MAX_METHODS 16

typedef struct {
  const char* method;
  int (*handler)(const char* params, char* response, int max_len);
} json_method_t;

// Global state
static char json_response_buffer[JSON_MAX_RESPONSE_SIZE];
static int json_response_length = 0;
static const json_method_t* json_registered_methods = NULL;

// String utilities
static inline int json_strcpy_safe(char* dst, const char* src, int max_len) {
  int i = 0;
  while (i < max_len - 1 && src[i] != '\0') {
    dst[i] = src[i];
    i++;
  }
  dst[i] = '\0';
  return i;
}

static inline int json_int_to_str(char* buf, int value, int max_len) {
  if (max_len < 12) return 0;
  char temp[12];
  int i = 0, len = 0;
  
  if (value == 0) {
    buf[0] = '0';
    buf[1] = '\0';
    return 1;
  }
  
  bool negative = value < 0;
  if (negative) value = -value;
  
  while (value > 0) {
    temp[i++] = '0' + (value % 10);
    value /= 10;
  }
  
  if (negative) buf[len++] = '-';
  while (i > 0) buf[len++] = temp[--i];
  buf[len] = '\0';
  return len;
}

// JSON parsing utilities
static inline int json_find_key(const char* json, const char* key) {
  int key_len = 0;
  while (key[key_len] != '\0') key_len++;
  
  for (int i = 0; json[i] != '\0'; i++) {
    if (json[i] == '"') {
      int j = 0;
      while (j < key_len && json[i + 1 + j] == key[j]) j++;
      if (j == key_len && json[i + 1 + j] == '"' && json[i + 2 + j] == ':') {
        return i + 3 + j;
      }
    }
  }
  return -1;
}

static inline int json_parse_string(const char* json, const char* key, char* value, int max_len) {
  int pos = json_find_key(json, key);
  if (pos < 0) return -1;
  
  while (json[pos] == ' ' || json[pos] == '\t') pos++;
  if (json[pos] != '"') return -1;
  pos++;
  
  int i = 0;
  while (i < max_len - 1 && json[pos] != '"' && json[pos] != '\0') {
    value[i++] = json[pos++];
  }
  value[i] = '\0';
  return i;
}

static inline int json_parse_int(const char* json, const char* key) {
  int pos = json_find_key(json, key);
  if (pos < 0) return -1;
  
  while (json[pos] == ' ' || json[pos] == '\t') pos++;
  
  int value = 0;
  bool negative = false;
  if (json[pos] == '-') {
    negative = true;
    pos++;
  }
  
  while (json[pos] >= '0' && json[pos] <= '9') {
    value = value * 10 + (json[pos] - '0');
    pos++;
  }
  
  return negative ? -value : value;
}

// Response builders
static inline int json_build_success(char* buffer, int max_len, const char* result) {
  char* p = buffer;
  p += json_strcpy_safe(p, "{\"result\":", max_len - (p - buffer));
  p += json_strcpy_safe(p, result, max_len - (p - buffer));
  p += json_strcpy_safe(p, ",\"error\":null}", max_len - (p - buffer));
  return p - buffer;
}

static inline int json_build_error(char* buffer, int max_len, const char* error, const char* message) {
  char* p = buffer;
  p += json_strcpy_safe(p, "{\"result\":null,\"error\":{\"code\":\"", max_len - (p - buffer));
  p += json_strcpy_safe(p, error, max_len - (p - buffer));
  p += json_strcpy_safe(p, "\",\"message\":\"", max_len - (p - buffer));
  p += json_strcpy_safe(p, message, max_len - (p - buffer));
  p += json_strcpy_safe(p, "\"}}", max_len - (p - buffer));
  return p - buffer;
}

// System info builder
static inline int json_build_system_info(char* buffer, int max_len, const char* device_name, const char* features) {
  char* p = buffer;
  p += json_strcpy_safe(p, "{\"device\":\"", max_len - (p - buffer));
  p += json_strcpy_safe(p, device_name, max_len - (p - buffer));
  p += json_strcpy_safe(p, "\",\"version\":\"", max_len - (p - buffer));
  p += json_strcpy_safe(p, (const char*)gitversion, max_len - (p - buffer));
  p += json_strcpy_safe(p, "\",\"features\":[", max_len - (p - buffer));
  p += json_strcpy_safe(p, features, max_len - (p - buffer));
  p += json_strcpy_safe(p, "]}", max_len - (p - buffer));
  return p - buffer;
}

// Core JSON RPC functions
static inline int json_rpc_init(const json_method_t* methods) {
  json_registered_methods = methods;
  json_response_length = 0;
  json_response_buffer[0] = '\0';
  return 0;
}

static inline int json_rpc_handle_request(const char* request, int len) {
  (void)len;
  
  char method[64];
  if (json_parse_string(request, "method", method, sizeof(method)) < 0) {
    json_response_length = json_build_error(json_response_buffer, sizeof(json_response_buffer),
      "parse_error", "Invalid JSON request");
    return 0;
  }
  
  // Find and call handler
  for (int i = 0; json_registered_methods[i].method != NULL; i++) {
    const char* reg_method = json_registered_methods[i].method;
    int j = 0;
    while (reg_method[j] != '\0' && method[j] != '\0' && reg_method[j] == method[j]) j++;
    
    if (reg_method[j] == '\0' && method[j] == '\0') {
      json_response_length = json_registered_methods[i].handler(request, json_response_buffer, sizeof(json_response_buffer));
      return 0;
    }
  }
  
  json_response_length = json_build_error(json_response_buffer, sizeof(json_response_buffer),
    "method_not_found", "Unknown method");
  return 0;
}

static inline int json_rpc_get_response(char* buffer, int offset, int max_len) {
  if (json_response_length == 0) {
    return 0;
  }
  
  if (offset >= json_response_length) return 0;
  
  int remaining = json_response_length - offset;
  int copy_len = (remaining < max_len) ? remaining : max_len;
  
  for (int i = 0; i < copy_len; i++) {
    buffer[i] = json_response_buffer[offset + i];
  }
  
  return copy_len;
}

static inline bool json_rpc_has_response(void) {
  return json_response_length > 0;
}

#endif