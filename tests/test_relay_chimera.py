#!/usr/bin/env python
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from firmware.python import Panda

p = Panda()

p.set_safety_mode(Panda.SAFETY_TOYOTA)

while True:
  p.send_heartbeat()
  time.sleep(0.1)

# p.set_safety_mode(Panda.SAFETY_NOOUTPUT)
