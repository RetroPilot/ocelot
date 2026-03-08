# RetroPilot JSON RPC API Reference

## Supported Devices
- **Relay Core** - GPIO and relay control
- **Chimera** - CAN gateway with sensor inputs
- **Interceptor Core** - Analog sensor interception (unconfigured mode only)
- **Actuator Core** - Motor control with multiple operating modes

## Universal Methods

### `system.info`
**Request:** 
```json
{"method": "system.info"}
```
**Response (Relay Core):** 
```json
{
  "result": {
    "device": "relay_core",
    "version": "v1.0.0-DEV-unknown-DEBUG", 
    "features": ["gpio", "relay"]
  },
  "error": null
}
```
**Response (Chimera):** 
```json
{
  "result": {
    "device": "chimera",
    "version": "v1.0.0-DEV-unknown-DEBUG", 
    "features": ["can_gateway", "sensors"]
  },
  "error": null
}
```
**Response (Interceptor Core):** 
```json
{
  "result": {
    "device": "interceptor_core",
    "version": "v1.0.0-DEV-unknown-DEBUG", 
    "features": ["adc", "dac", "mode"]
  },
  "error": null
}
```
**Response (Actuator Core):** 
```json
{
  "result": {
    "device": "actuator_core",
    "version": "v1.0.0-DEV-unknown-DEBUG", 
    "features": ["motor1", "motor2", "relay", "adc"]
  },
  "error": null
}
```

### `system.methods`
**Request:** 
```json
{"method": "system.methods"}
```
**Response (Actuator Core):**
```json
{
  "result": ["system.info", "system.methods", "system.reset", "actuator.get_status", "actuator.motor", "actuator.relay"],
  "error": null
}
```
**Response (Interceptor Core):**
```json
{
  "result": ["system.info", "system.methods", "system.reset", "interceptor.get_values", "interceptor.set_dac", "interceptor.emergency_stop"],
  "error": null
}
```
**Response (Relay Core):**
```json
{
  "result": ["system.info", "system.methods", "system.reset", "relay.set"],
  "error": null
}
```

### `system.reset`
**Request:** 
```json
{"method": "system.reset"}
```
**Response:** Device resets immediately

## Relay Core Methods

### `relay.set`
**Request (set relay):** 
```json
{"method": "relay.set", "params": {"relay": 3, "state": 1}}
```
**Response (set relay):** 
```json
{"result": {"success": true, "relay_ctrl": [0,0,1,0,0,0,0,0], "gpio": [1,1,1,1,1,1,1,1]}, "error": null}
```
**Request (poll state):** 
```json
{"method": "relay.set"}
```
**Response (poll state):** 
```json
{"result": {"relay_ctrl": [0,0,1,0,0,0,0,0], "gpio": [1,1,1,1,1,1,1,1]}, "error": null}
```

### `flash.format`
**Request:** 
```json
{"method": "flash.format"}
```
**Response:** 
```json
{"result": "formatted", "error": null}
```

### `flash.write`
**Request:** 
```json
{"method": "flash.write", "params": {"index": 1, "cfg_type": 5, "gpio_en": 1, "gpio_in": 15}}
```
**Response:** 
```json
{"result": "written", "error": null}
```

### `flash.read`
**Request:** 
```json
{"method": "flash.read", "params": {"index": 1}}
```
**Response:** 
```json
{"result": {"index": 1, "cfg_type": 5, "gpio_en": 1, "gpio_in": 15}, "error": null}
```

## Chimera Methods

### `chimera.get_values`
**Request:** 
```json
{"method": "chimera.get_values"}
```
**Response:** 
```json
{
  "result": {
    "steer_angle": 0.0,
    "steer_rate": 0.0,
    "engine_rpm": 800,
    "ignition": true,
    "brake_pressed": false,
    "vehicle_speed": 25.5,
    "adc0": 2048,
    "adc1": 1024,
    "buttons": 0
  },
  "error": null
}
```

## Interceptor Core Methods

### `interceptor.get_values`
**Request:** 
```json
{"method": "interceptor.get_values"}
```
**Response:** 
```json
{
  "result": {
    "adc0": 2048,
    "adc1": 1024,
    "dac0": 2000,
    "dac1": 2000,
    "mode": 0,
    "state": 0,
    "enabled": false
  },
  "error": null
}
```

### `interceptor.set_dac`
**Request:** 
```json
{"method": "interceptor.set_dac", "params": {"d0": 2500, "d1": 1800}}
```
**Response (Success):** 
```json
{
  "result": {
    "dac0": 2500,
    "dac1": 1800
  },
  "error": null
}
```
**Response (Mode Restricted):** 
```json
{
  "result": null,
  "error": {
    "code": "mode_restricted",
    "message": "DAC control only available in unconfigured mode"
  }
}
```

**Notes:**
- **DAC parameters:** `d0`=DAC channel 0, `d1`=DAC channel 1 (0-4095 range)
- DAC control is only available when Interceptor Core is in unconfigured mode (mode 0)
- Control automatically times out after ~4 timer cycles, reverting to ADC pass-through
- Mode values: 0=Unconfigured, 1=Differential, 2=Gas Pedal
- **Parameter abbreviations:** Used to keep commands under 64-byte USB packet limit

## Actuator Core Methods

### `actuator.get_status`
**Request:** 
```json
{"method": "actuator.get_status"}
```
**Response:** 
```json
{
  "result": {
    "motor1_pwm": 75,
    "motor2_pwm": 50,
    "motor1_dir": 1,
    "motor2_dir": 0,
    "motor1_enable": true,
    "motor2_enable": true,
    "relay_state": true,
    "adc0": 2048,
    "adc1": 1024,
    "state": 0,
    "mode": 0,
    "usb_ctrl": true
  },
  "error": null
}
```

### `actuator.motor`
**Request:** 
```json
{"method": "actuator.motor", "params": {"m": 1, "p": 75, "d": 1}}
```
**Response:** 
```json
{
  "result": "ok",
  "error": null
}
```

### `actuator.relay`
**Request:** 
```json
{"method": "actuator.relay", "params": {"value": 1}}
```
**Response:** 
```json
{
  "result": "ok",
  "error": null
}
```

**Notes:**
- **Motor parameters:** `m`=motor number (1-2), `p`=PWM (0-100%), `d`=direction (0=reverse, 1=forward)
- **Relay parameters:** `value`=relay state (0=off, 1=on)
- **Safety timeout:** Motors and relay automatically disable after 1 second without commands
- **Individual control:** Each motor controlled separately for precise operation
- **Parameter abbreviations:** Used to keep commands under 64-byte USB packet limit

## Common Error Codes

### `method_not_found`
```json
{
  "result": null,
  "error": {
    "code": "method_not_found",
    "message": "Unknown method"
  }
}
```

### `invalid_params`
```json
{
  "result": null,
  "error": {
    "code": "invalid_params",
    "message": "Parameter values out of range or invalid"
  }
}
```

### `mode_restricted`
```json
{
  "result": null,
  "error": {
    "code": "mode_restricted",
    "message": "Operation not allowed in current device mode"
  }
}
```

### `parse_error`
```json
{
  "result": null,
  "error": {
    "code": "parse_error",
    "message": "Invalid JSON request"
  }
}
```