# Welcome to Ocelot
something, something marketing. do we need to do this? if you don't know, prepare to know.

this is the firmware repository for RetroPilot's family of automotive gateway and control modules.

they allow users to easily retrofit parts from other cars, selectively gateway ECUs at will, filter and convert messages, control actuators, and more.

## Ocleot and Chimera

**[Gateway V2](https://shop.retropilot.org/product/gateway-v2/)** - the CAN gateway of your dreams. gateway messages between buses, retrofit parts, etc.
- gateway and modify CAN messages between multiple buses
- translate signals and convert message formats for part retrofitting
- filter and manipulate specific bits in CAN frames
- enable features by modifying ECU communications
- integrated SSR for connecting and disconnecting busses on-the-fly

**[Chimera](https://shop.retropilot.org/product/chimera/)** - CAN gateway with raw sensor inputs for complete vehicle integration
- has raw sensor inputs for VSS (vehicle speed), CPS (crank position sensor), and ADC (for cruise control stalks)
- can convert existing CAN messages to RetroPilot format for cars that already have some info available over CAN
- has OBD-C connector on-board for connecting to Comma devices
- has USB port on-board and panda-compatible API for sending/recieving messages over CAN
- ideal for vehicles that need CAN gatewaying and/or direct sensor processing in one device

## RetroPilot Cores

**[Actuator Core](https://shop.retropilot.org/product/actuator-core/)** - motor control for steering, throttle, or brake actuators and other automotive applications
- control two DC motors up to 6A each
- build custom actuators for steering, gas, brakes and more

**[Relay Core](https://shop.retropilot.org/product/relay-core/)** - GPIO and relay control with CAN integration
- control lights, fans, pumps, or other 12V devices via CAN messages or GPIO switches
- build custom lighting controllers that respond to vehicle state
- add remote-controlled accessories to your vehicle

**[Interceptor Core](https://shop.retropilot.org/product/interceptor-core/)** - analog sensor interception and injection
- intercept gas pedal signals and modify them for drive-by-wire applications
- modify torque sensor signals for custom steering feel
- add safety limits or filtering to analog sensor inputs

**⚠️ WARNING: Interceptor Core for steering applications is for RESEARCH AND DEVELOPMENT ONLY. Do not use on public roads. Use at your own risk with extreme caution, controlled environments, and always have backup safety systems. RetroPilot assumes no liability for any damages, injuries, or accidents.**

## Hardware
hardware is based on STM32F413 (most modules) or STM32F205 (pedal), and is fully OSHW where applicable. board files, bom, and schematics are located in the `hardware` folder.

## Firmware
firmware borrows heavily from [panda](https://github.com/commaai/panda), and should be firmware-compatible with Panda firmware as a black panda. conversely, most firmware should also work on a panda, which is helpful for development.

### Available Projects:
- `chimera` - Chimera gateway firmware
- `actuator_core` - motor control
- `relay_core` - relay and GPIO control  
- `interceptor_core` - analog sensor interception
- `ibst` - iBooster control
- `smart_dsu` - smart DSU module
- `eps_gw` - EPS gateway
- `steer_actuator_demo` - steering actuator demo
- `panda` - panda compatibility mode
- `pedal` - pedal module (STM32F205)

### Prerequisites:

#### Docker Setup (recommended):
install Docker for your operating system. then build the container:
```
./build_container.sh
```

#### Python Setup (for configuration tool):
install pipenv:
```
pip install pipenv
```
then setup the environment:
```
pipenv install
```

### Flashing:

Docker (recommended):
enter the environment
```
pipenv shell
```
you will want to do this any time you compile firmware, flash, or use the configuration tool. flash the board with
```
./flash_docker.sh <project name>
```

where <project_name> is the board or project you want to flash, for example actuator_core for Actuator Core. providing no arguments will list all projects available to build and flash.

### Configuration tool:

#### Mac/Linux:
enter the pipenv environment:
```
pipenv shell
```
then run the configuration tool:
```
./stm_flash_config.py
```
select the device you'd like to configure and follow the wizard.

##### Configuration by Device:

#### Actuator Core Configuration:

**System Config** - set device operating mode:
- `debug_lvl`: debug output level
- `can_out_en`: enable CAN message transmission
- `iwdg_en`: enable watchdog timer (0 while configuring, 1 when finished)
- `mode`: operating mode (0=Default/USB control, 1=Steer, 2=Cruise)

**TPS Config** - throttle position sensor setup (cruise mode only):
- `adc1`: minimum throttle position (fully closed)
- `adc2`: maximum throttle position (fully open)
- `adc_tolerance`: position control deadzone
- `adc_num`: ADC channel number (0 or 1)

**Motor Config** - throttle motor setup:
- `bridge_channel`: which H-bridge to use (1 or 2)
- `type`: motor function (1=throttle motor)
- `polarity`: motor direction (1=normal, 2=inverted)

**Clutch Config** - clutch motor setup:
- `bridge_channel`: which H-bridge to use (1 or 2)
- `type`: motor function (2=clutch)
- `polarity`: motor direction (1=normal, 2=inverted)

#### Interceptor Core Configuration:

**System Config** - set device operating mode:
- `debug_lvl`: debug output level
- `can_out_en`: enable CAN message transmission
- `iwdg_en`: enable watchdog timer (0 while configuring, 1 when finished)
- `mode`: operating mode (0=Unconfigured, 1=Differential, 2=Gas Pedal)
- `override_threshold`: differential mode threshold (default 336)

**ADC Channel Validation** - sensor validation setup:

*Differential Mode (center point + tolerance):*
- `adc1`: center value (expected sensor position)
- `adc2`: unused (set to 0)
- `adc_tolerance`: deviation tolerance (±)

*Gas Pedal Mode (range limits):*
- `adc1`: maximum valid value (e.g., 4000 for 4.0V)
- `adc2`: minimum valid value (e.g., 200 for 0.2V)
- `adc_tolerance`: hysteresis/noise tolerance

#### Relay Core Configuration:

**System Config** - basic system setup:
- `debug_lvl`: debug output level
- `can_out_en`: enable CAN message transmission
- `iwdg_en`: enable watchdog timer (0 while configuring, 1 when finished)

**Relay Configs** - individual relay setup:
- `label`: predefined automotive function (TURN_L_FRONT, HEAD_L, BRAKE_L, etc.)
- `gpio_en`: enable GPIO control (0 or 1)
- `gpio_bitmask`: GPIO pin combination required
- `can_addr`: CAN message ID to monitor (0 disables)
- `can_cmp_val`: CAN signal value that triggers relay
- `sig_len`/`shift_amt`: CAN signal extraction parameters

#### Chimera Configuration:

**System Config** - basic system setup:
- `debug_lvl`: debug output level
- `can_out_en`: enable CAN message transmission
- `iwdg_en`: enable watchdog timer (0 while configuring, 1 when finished)

**CAN Signal Configs** - gateway message processing:
- `can_id`: which CAN message to process (hex like 0x123)
- `shift_amt`/`sig_len`: which bits contain your signal (shift right X bits, then mask Y bits)
- `scale_mult`/`scale_offs`: math to convert raw values (result = raw * mult + offset)
- `endian_type`: byte order (0=little endian, 1=big endian)
- `is_signed`: treat signal as signed integer or not
- `enabled`: enable this signal processing (0 or 1)

**HALL Sensor Configs** - vehicle speed and engine sensor processing:
- `vss_ppd`: pulses per distance (mile or km)
- `is_km`: use metric units (1=km, 0=miles)
- `rel_cnt`: reluctor tooth count for crank position sensor
- `skipped_teeth`: missing teeth count (e.g., 36-1 = 1 skipped tooth)

**ADC Button Configs** - cruise control button detection:
- `adc1`/`adc2`: threshold values for button states
- `adc_tolerance`: noise tolerance
- `adc_num`: ADC channel for cruise control buttons

#### Any OS:
use the web-based config tool available [here](https://coreconfig.retropilot.org/)

## Documentation
full documentation and examples are available at the [RetroPilot Wiki](https://wiki.retropilot.org/index/hardware/ocelot):

https://wiki.retropilot.org/index/hardware/ocelot
