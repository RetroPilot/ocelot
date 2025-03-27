#!/usr/bin/env sh
set -e

DFU_UTIL="dfu-util"

cd ../..
ACTUATOR_CORE=1 scons -u
cd RetroPilot_Cores/actuator_core

$DFU_UTIL -d 0483:df11 -a 0 -s 0x08004000 -D ../../obj/actuator_core.bin.signed
$DFU_UTIL -d 0483:df11 -a 0 -s 0x08000000:leave -D ../../obj/bootstub.actuator_core.bin
