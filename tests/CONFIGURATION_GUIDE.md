# RetroPilot Device Configuration Guide

## Configuration Overview

All RetroPilot devices use a flash-based configuration system with standardized parameter structures. Configuration can be done via:
- **Configuration tool**: `./stm_flash_config.py` (Mac/Linux)
- **Web tool**: https://coreconfig.retropilot.org (Any OS)
- **WebUSB flash tool**: https://flash.retropilot.org

## Universal System Configuration (Index 0)

All devices require a system configuration at index 0:

| Parameter | Type | Description | Values |
|-----------|------|-------------|---------|
| `debug_lvl` | uint8 | Debug output level | 0-255 (0=off, 1=basic, 2=verbose) |
| `can_out_en` | uint8 | Enable CAN message transmission | 0=disabled, 1=enabled |
| `iwdg_en` | uint8 | Enable watchdog timer | 0=disabled (config mode), 1=enabled (production) |
| `mode` | uint8 | Device operating mode | Device-specific values |

**⚠️ Important**: Set `iwdg_en = 0` while configuring, then set to `1` when finished to prevent watchdog resets.

---

## Actuator Core Configuration

### System Config (Index 0)
| Parameter | Values | Description |
|-----------|--------|-------------|
| `mode` | 0=Default/USB, 1=Steer, 2=Cruise | Operating mode |

### Motor Config (Indices 1-2)
Configure H-bridge channels for different motor functions:

| Parameter | Type | Description | Values |
|-----------|------|-------------|---------|
| `bridge_channel` | uint8 | H-bridge to use | 1 or 2 |
| `type` | uint8 | Motor function | 1=throttle, 2=clutch |
| `polarity` | uint8 | Motor direction | 1=normal, 2=inverted |

### TPS Config (Index 3) - Cruise Mode Only
Throttle position sensor setup:

| Parameter | Type | Description | Range |
|-----------|------|-------------|-------|
| `adc1` | uint32 | Minimum throttle (closed) | 0-4095 |
| `adc2` | uint32 | Maximum throttle (open) | 0-4095 |
| `adc_tolerance` | uint16 | Position control deadzone | 1-500 |
| `adc_num` | uint8 | ADC channel number | 0 or 1 |

**Example Configuration:**
```
System: mode=2, can_out_en=1, iwdg_en=1
Motor 1: bridge_channel=1, type=1, polarity=1  # Throttle motor
Motor 2: bridge_channel=2, type=2, polarity=1  # Clutch motor
TPS: adc1=200, adc2=3800, adc_tolerance=50, adc_num=0
```

---

## Interceptor Core Configuration

### System Config (Index 0)
| Parameter | Values | Description |
|-----------|--------|-------------|
| `mode` | 0=Unconfigured, 1=Differential, 2=Gas Pedal | Operating mode |
| `override_threshold` | Default: 336 | Differential mode threshold |

### ADC Channel Validation (Indices 1-2)

#### Differential Mode (Center Point + Tolerance)
| Parameter | Description | Example |
|-----------|-------------|---------|
| `adc1` | Center value (expected position) | 2048 |
| `adc2` | Unused (set to 0) | 0 |
| `adc_tolerance` | Deviation tolerance (±) | 100 |

#### Gas Pedal Mode (Range Limits)
| Parameter | Description | Example |
|-----------|-------------|---------|
| `adc1` | Maximum valid value | 4000 (4.0V) |
| `adc2` | Minimum valid value | 200 (0.2V) |
| `adc_tolerance` | Hysteresis/noise tolerance | 50 |

**Example Configurations:**

*Differential Mode:*
```
System: mode=1, can_out_en=1, iwdg_en=1, override_threshold=336
ADC0: adc1=2048, adc2=0, adc_tolerance=100, adc_en=1
ADC1: adc1=2048, adc2=0, adc_tolerance=100, adc_en=1
```

*Gas Pedal Mode:*
```
System: mode=2, can_out_en=1, iwdg_en=1
ADC0: adc1=4000, adc2=200, adc_tolerance=50, adc_en=1
ADC1: adc1=4000, adc2=200, adc_tolerance=50, adc_en=1
```

---

## Relay Core Configuration

### System Config (Index 0)
Standard system configuration only.

### Relay Configs (Indices 1-8)
Individual relay setup for automotive functions:

| Parameter | Type | Description | Values |
|-----------|------|-------------|---------|
| `label` | enum | Predefined function | See automotive labels below |
| `gpio_en` | uint8 | Enable GPIO control | 0=disabled, 1=enabled |
| `gpio_bitmask` | uint8 | GPIO pin combination | Bit pattern (0-255) |
| `can_addr` | uint8 | CAN message ID to monitor | 0=disabled, 1-255=CAN ID |
| `can_cmp_val` | uint32 | CAN signal value trigger | Value to match |
| `sig_len` | uint8 | CAN signal length (bits) | 1-32 |
| `shift_amt` | uint8 | CAN signal bit shift | 0-63 |

#### Automotive Label Options:
- `TURN_L_FRONT`, `TURN_L_REAR` - Left turn signals
- `TURN_R_FRONT`, `TURN_R_REAR` - Right turn signals  
- `HEAD_L`, `HEAD_R` - Headlights
- `BRAKE_L`, `BRAKE_R` - Brake lights
- `REVERSE_L`, `REVERSE_R` - Reverse lights
- `FOG_FRONT`, `FOG_REAR` - Fog lights
- `HAZARD` - Hazard flashers

**Example Configuration:**
```
Relay 1: label=HEAD_L, gpio_en=1, gpio_bitmask=0x01, can_addr=0x123, can_cmp_val=1
Relay 2: label=BRAKE_L, gpio_en=0, can_addr=0x456, can_cmp_val=1, sig_len=1, shift_amt=0
```

---

## Chimera Configuration

### System Config (Index 0)
Standard system configuration only.

### CAN Signal Configs (Indices 1-16)
Gateway message processing:

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| `can_id` | uint32 | CAN message ID (hex) | 0x123 |
| `shift_amt` | uint8 | DBC start bit (LSB for little endian, MSB for big endian) | 8 |
| `sig_len` | uint8 | Signal length in bits | 16 |
| `scale_factor` | float | Scale factor | 0.1 |
| `scale_offset` | float | Scale offset | -100.0 |
| `endian_type` | uint8 | Byte order | 0=little, 1=big |
| `is_signed` | uint8 | Signed integer | 0=unsigned, 1=signed |
| `enabled` | uint8 | Enable processing | 0=disabled, 1=enabled |

**Formula**: `result = (raw_value * scale_factor) + scale_offset`

### HALL Sensor Configs (Index 17)
Vehicle speed and engine sensor processing:

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| `vss_ppd` | uint16 | Pulses per distance | 4000 (pulses/mile) |
| `is_km` | uint8 | Use metric units | 0=miles, 1=km |
| `rel_cnt` | uint8 | Reluctor tooth count | 36 |
| `skipped_teeth` | uint8 | Missing teeth count | 1 (for 36-1) |

### ADC Button Configs (Index 18)
Cruise control button detection:

| Parameter | Type | Description | Range |
|-----------|------|-------------|-------|
| `adc1` | uint32 | Button threshold 1 | 0-4095 |
| `adc2` | uint32 | Button threshold 2 | 0-4095 |
| `adc_tolerance` | uint16 | Noise tolerance | 1-200 |
| `adc_num` | uint8 | ADC channel | 0-15 |

**Example Configuration:**
```
System: can_out_en=1, iwdg_en=1
CAN Signal 1: can_id=0x123, shift_amt=0, sig_len=16, scale_factor=1.0, scale_offset=0.0, enabled=1
CAN Signal 2: can_id=0x456, shift_amt=8, sig_len=8, scale_factor=0.1, scale_offset=-50.0, enabled=1
HALL: vss_ppd=4000, is_km=0, rel_cnt=36, skipped_teeth=1
ADC Buttons: adc1=1000, adc2=2000, adc_tolerance=50, adc_num=0
```

---

## Configuration Process

### 1. Format Flash (First Time)
```bash
pipenv shell
./stm_flash_config.py
# Select device and choose "Format Flash"
```

### 2. Configure System (Index 0)
Always configure system settings first:
- Set `iwdg_en = 0` during configuration
- Set appropriate `mode` for device
- Enable `can_out_en = 1` if CAN output needed

### 3. Configure Device-Specific Settings
Follow device-specific configuration sections above.

### 4. Enable Watchdog
Set `iwdg_en = 1` in system config when finished.

### 5. Flash Firmware
```bash
./flash_docker.sh <device_name>
```

## Troubleshooting

### No CAN Output
1. Check `can_out_en = 1` in system config
2. Verify device is not in unconfigured mode
3. Check for fault states in debug output
4. Ensure CAN bus speed matches (typically 500k or 250k)

### Device Not Responding
1. Check `iwdg_en = 0` during configuration
2. Verify flash is formatted
3. Check USB connection and drivers
4. Try entering bootloader mode

### Invalid Configuration
1. Verify all required indices are configured
2. Check parameter ranges and types
3. Ensure CAN IDs don't conflict
4. Validate ADC ranges are realistic

## Safety Notes

⚠️ **CRITICAL**: Interceptor Core for steering applications is for RESEARCH AND DEVELOPMENT ONLY. Do not use on public roads.

⚠️ **WATCHDOG**: Always set `iwdg_en = 0` while configuring to prevent resets.

⚠️ **VALIDATION**: Test all configurations in safe, controlled environments before deployment.
