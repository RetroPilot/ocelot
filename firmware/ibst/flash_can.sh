#!/usr/bin/env sh
set -e

cd ..
IBST=1 scons -u
cd ibst

../../tests/gateway/enter_canloader.py ../obj/panda.bin.signed
