### Parts

| QTY | PN        | Description           | 
|-----|-----------|-----------------------|
|1    |BTS7960    | motor driver          |
|2    |FIT0493    | 350RPM motor          |
|1    |6656K236   | Ultra-Thin Ball Bearing, Steel Ring, Shielded, 60mm Shaft Diameter and 78mm Housing ID  |
|1    |Ocelot Pro | Ocelot Pro CAN Devkit |
|1    |89245-0K020| TOYOTA angle sensor   |
|1    | -         | 1kOhm Resistor        |
|1    | -         | toggle switch         |

STL Files are in [here](../../hardware/actuator-demo) and were designed by David Abraham (@Vasillo on Discord)

Gear ratio between the motors and final drive gear is 5:1, thus the final torque output is around 9.414384 Nm at peak `((1.176798 Nm /motor * 2 motors) * 0.80 * 5)`

The 1k resistor pulls PB14 up to 3.3v, and the switch pulls the same pin to GND.

### Wiring

|  PIN  |  Function  |
|-------|------------|
| PB12  | L_ENABLE   |
| PB13  | R_ENABLE   |
| PB14  | SWITCH IN  |
| PA2   | L_PWM      |
| PA3   | R_PWM      |

Constant 12V should be used to power the Hbridge VBatt inputs, and a switched 12V should be used to connect the Ocelot Pro's 12V input to power.

#### Angle Sensor
CANL and CANH should be on the same bus as the Ocelot Pro CAN1. Below is a wiring guide.

![Angle Sensor Pinout](assets/pinout.png)

### CAN

This example only uses CAN1 to interface with CAN. Make sure to terminate the bus with a 120 Ohm resistor so everything communicates correctly.

#### Inputs:
0x22E - torque / angle request (angle req is not yet implemented)

#### Outputs:
0x22F - actuator status
0x22D - fake cruise state to engage OP demo

#### DBC Sample

```dbc
BO_ 741 STEERING_COMMAND: 5 EON
 SG_ REQUESTED_STEER_TORQUE : 29|11@1- (0.5,0) [-512|511.5] "Nm" EPAS
 SG_ COUNTER : 8|4@1+ (1,0) [0|15] "" EPAS
 SG_ REQUESTED_STEER_ANGLE : 14|15@1- (0.1,0) [-1638.4|1638.3] "deg" EPAS
 SG_ STEER_MODE : 12|2@1+ (1,0) [0|3] "" EPAS
 SG_ CHECKSUM : 0|8@1+ (1,0) [0|255] "" EPAS

BO_ 608 STEERING_STATUS: 5 EPAS
 SG_ COUNTER : 8|4@1+ (1,0) [0|15] "" EON
 SG_ STATUS : 12|4@1+ (1,0) [0|15] "" EON
 SG_ CHECKSUM : 0|8@1+ (1,0) [0|255] "" EON
 SG_ STEERING_TORQUE_EPS : 29|11@1- (0.8,0) [-819.2|818.4] "Nm" EON
 SG_ STEERING_TORQUE_DRIVER : 18|11@1- (0.8,0) [-819.2|818.4] "Nm" EON
 SG_ STEERING_OK : 16|1@1+ (1,0) [0|1] "" EON

BO_ 37 TOYOTA_STEERING_ANGLE_SENSOR1: 8 TOYOTASAS
 SG_ TOYOTA_STEER_RATE : 35|12@0- (1,0) [-2000|2000] "deg/s" EON
 SG_ TOYOTA_STEER_FRACTION : 39|4@0- (0.1,0) [-0.7|0.7] "deg" EON
 SG_ TOYOTA_STEER_ANGLE : 3|12@0- (1.5,0) [-500|500] "deg" EON

BO_ 870 CURRENT_STATE: 4 EON
 SG_ ENABLED : 8|1@1+ (1,0) [0|0] ""  HIM
 SG_ SET_SPEED : 16|8@1+ (1,0) [0|255] "mph"  HIM
 SG_ CURRENT_SPEED : 24|8@1+ (1,0) [0|255] "kph" IBST
 SG_ CHECKSUM : 0|8@1+ (1,0) [0|255] ""  HIM
```

### Openpilot
**Note: DO NOT UNDER ANY CIRCUMSTANCES INSTALL THIS ON A CAR. DO NOT ATTEMPT TO DRIVE WITH ANY OF THE HARDWARE OR SOFTWARE USED IN THIS DEMO. THIS DEMO IS RELEASED FOR RESEARCH PURPOSES ONLY, AND IS NOT MEANT TO BE USED ON PUBLIC ROADS. ALL INFORMATION AND SOFTWARE IS PROVIDED AS-IS.**

This demo uses an older version of OP that runs on the Eon/C2 or a PC. To connect it in, you will need a Panda (white/grey/black is fine) or a second Ocelot Pro with Panda Shield, relay, and dev harness.

Connect the 12Vbat line to 12V, and ignition to the same switched 12V ignition as the Ocelot. CAN1 and CAN3 from the Panda are bridged together and connected to the CAN bus containing the actuator and the Toyota SAS.

Install the following on an Eon or C2 device:

https://github.com/RetroPilot/openpilot

and checkout branch `actuator_demo`
