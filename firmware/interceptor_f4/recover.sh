#!/usr/bin/env sh
set -e

DFU_UTIL="dfu-util"

cd ..
INTERCEPTOR_F4=1 scons -u
cd interceptor_f4

$DFU_UTIL -d 0483:df11 -a 0 -s 0x08004000 -D ../obj/interceptor_f4.bin.signed
$DFU_UTIL -d 0483:df11 -a 0 -s 0x08000000:leave -D ../obj/bootstub.interceptor_f4.bin
