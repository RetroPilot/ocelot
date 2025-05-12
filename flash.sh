#!/usr/bin/env sh
set -e

scons -u $1
PYTHONPATH=.. python3 -c "from firmware/python import Panda; Panda().flash('firmware/obj/$1.bin.signed')"
