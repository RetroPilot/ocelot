# Ocelot Pro GPIO example
## Introduction
This code functions as sort of an introduction to Ocelot / Panda / STM32 bare metal programming. The purpose is to show users how to set up interfaces on this platform using the Comma AI Panda codebase as a skeleton to build things on top. Maybe you want to build a CAN controlled relay and install it in a car. How would you go about doing that? Well a good place to start would be [Comma's Panda code](https://github.com/commaai/panda), since it aims to be compliant with MISRA and potentially ASIL. After that, it's a matter of grabbing an [Ocelot Pro](https://shop.retropilot.org/product/ocelot-pro-devkit/), setting up GPIOs, then connecting all that to the CAN interface so the GPIO responds to your custom CAN message.

That is where this example will hopefully help you out.

## What is covered here
We will do the following here:
- CAN interfaces setup (CAN1, CAN2, CAN3)
- Timer setup
- PWM setup
- GPIO setup

You will want a decent working knowledge of how to read code, preferably C. I'm not sure this would classify as a "beginner guide" per se, but if you're familiar with Arduino to some extent this should be somewhat approachable. We will not go too far in depth in terms of how the registers are actually set, nor what memory addresses and individual bits are actually set. So I guess difficulty: intermediate? Nothing to be scared of!

## Pins

PA2 - PWM (TIM5_CH3)
PA3 - PWM (TIM5_CH4)
PB12 - GPIO (digital out)
PB13 - GPIO (digital out)

Connect up to a scope, LEDs, h-bridge, whatever. Make sure to connect via a resistor (like 220 ohm or something) if using LEDs.