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

### Flashing:

on Docker (recommended) 
```
./build_container.sh
./flash_docker.sh <project name> #build and flash after
```

on Ubuntu 20.04
```
./build_project.sh <project_name>
./recover.sh <project name>
```

providing no arguments will list all projects available to build and flash.

### Configuration tool:

#### Any OS:
use the web-based config tool available [here](https://coreconfig.retropilot.org/)

or, use the python-based console tool:

#### Mac/Linux:
install pipenv. setup the environment with:
`pipenv install` 
then enter the environment with:
`pipenv shell`
once inside, you can run `./stm_flash_config.py` to configure your device. select the device you'd like to configure and follow the wizard.

##### config tool options:

**CAN**: gateway and modify CAN messages. extract signals from specific bit positions, scale values, flip endianness, whatever you need.
- `can_id`: which CAN message to process (hex like 0x123)
- `shift_amt`/`sig_len`: which bits contain your signal (shift right X bits, then mask Y bits)
- `scale_mult`/`scale_offs`: math to convert raw values (result = raw * mult + offset). scale_mult must be 10x actual value (e.g. use 15 for 1.5x scaling)
- `endian_type`: byte order (0=little endian, 1=big endian)
- `is_signed`: treat signal as signed integer or not

**ADC**: monitor analog inputs with thresholds. useful for voltage/current sensing or any analog signal you want to watch.
- `adc1`/`adc2`: threshold values to trigger on
- `adc_tolerance`: how much the reading can vary before triggering
- `adc_num`: which ADC channel to monitor (0 or 1)

**RELAY**: control relays via GPIO pins or CAN messages. use bitmasks to combine multiple GPIO inputs, or trigger relays based on specific CAN signal values.
- `gpio_en`: enable GPIO control (1) or not (0)
- `gpio_bitmask`: which GPIO pins must be high to activate relay (bit 0 = pin 0, etc.)
- `can_addr`: CAN message ID to listen for relay commands (0 disables)
- `can_cmp_val`: specific CAN signal value that triggers the relay

**SYS**: system stuff like debug levels, CAN output enable/disable, watchdog timer settings.
- `debug_lvl`: how much debug info to output
- `can_out_en`: enable CAN message transmission
- `iwdg_en`: enable watchdog timer. set to 0 if you need to write more flash params, set to 1 when you are finished configuring the device

**VSS**: vehicle speed signal processing. configure pulses per mile/km and unit conversion for speed calculation.
- `vss_ppd`: pulses per distance (mile or km)
- `is_km`: use metric (1) or imperial (0) units
- `rel_cnt`: reluctor tooth count for engine CPS
- `skipped_tooth`: number of skipped teeth. for example 36-1 means 1 skipped tooth


## Documentation
full documentation and examples are available at the [RetroPilot Wiki](https://wiki.retropilot.org/index/hardware/ocelot):

https://wiki.retropilot.org/index/hardware/ocelot
