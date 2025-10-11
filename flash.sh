#!/usr/bin/env sh
set -e

PYTHONPATH=.. python3 -c "from firmware/python import Panda; Panda().flash('firmware/obj/$1.bin.signed')"
