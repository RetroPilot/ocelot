# SmartDSU

## Supported cars
- 2017-2019 Toyota Corolla
- 2016-2018 Toyota Rav4 (and Hybrid)
- 2016-2019 Toyota Prius
- other DSU-equipped Toyotas with 0x343 as the ACC message

## Supported cars with slight firmware changes
- 2018 Camry Hybrid (tested by @halfcocked)

## How it Works
the Toyota ACC message 0x343 is read in on bus 3 of the gateway, modified, re-checksummed, then sent on to bus 1. if the ACC control message 0x343 is detected on bus 1, it will be blocked from bus 3 entirely. if the alt ACC control message 0xF10 is detected, the smartDSU will modify the forwarded 0x343 with data from inside 0xF10. this method ensures perfect message send rate preservation between the DSU and the vehicle CAN bus. 

a similar approach is taken with AEB message 0x344. if 0xF11 is detected on bus 1, the device will modify the Toyota AEB message, allowing for non-acc brake control without disturbing the stock AEB system.

all other messages are forwarded as-is between the DSU and the car.

if the device detects that 0xF10, 0xF11, or 0x343 is not sent, it will timeout functionality accordingly and restore forwarding between the DSU and the vehicle.

## Installation

install the device with bus 1 facing the CAR CAN and bus 3 facing the DSU or radar. ensure bus 3 is terminated by applying a blob of solder to JL2 (on ocelot pro) or the solder bridge next to CAN3 (on ocelot gateway.)

refer to the hardware guide for more info:
https://wiki.retropilot.org/index/hardware/ocelot/hardware


## Modifying for cars that do not require PERMIT_BRAKING and others
in main.c the following lines of code must be modified to prevent modification of the ACC message since it will be handled by Openpilot instead:

```c
          if(dat[7] == toyota_checksum(address, dat, 8)) {
            // add permit_braking and recompute the checksum
            dat[2] &= 0x3F; // mask off the top 2 bits
            dat[2] |= (1 << 6U); // SET_ME_X01
            dat[3] |= (1 << 6U); // permit_braking
            dat[7] = toyota_checksum(address, dat, 8); 
```

simply comment out the mask and the bitwise math:

```c
          if(dat[7] == toyota_checksum(address, dat, 8)) {
            // add permit_braking and recompute the checksum
            //dat[2] &= 0x3F; // mask off the top 2 bits
            //dat[2] |= (1 << 6U); // SET_ME_X01
            //dat[3] |= (1 << 6U); // permit_braking
            //dat[7] = toyota_checksum(address, dat, 8); 
```

and flash the firmware
