#!/usr/bin/env sh
set -e

cd ..
SDSU=1 scons -u
cd smart_dsu

../../tests/gateway/enter_canloader.py ../obj/panda.bin.signed
