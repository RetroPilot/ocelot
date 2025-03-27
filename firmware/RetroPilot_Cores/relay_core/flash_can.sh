#!/usr/bin/env sh
set -e

cd ..
GATEWAY=1 GPIO_EXAMPLE=1 scons -u
cd gpio_example

../../tests/gateway/enter_canloader.py ../obj/panda.bin.signed
