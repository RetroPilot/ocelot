#!/usr/bin/env python3

import os
import sys
import time
import select
import codecs

from firmware.python import Panda

setcolor = ["\033[1;32;40m", "\033[1;31;40m"]
unsetcolor = "\033[00m"

port_number = int(os.getenv("PORT", "0"))
claim = os.getenv("CLAIM") is not None
no_color = os.getenv("NO_COLOR") is not None
no_reconnect = os.getenv("NO_RECONNECT") is not None

if __name__ == "__main__":
  while True:
    try:
      serials = Panda.list()
      if os.getenv("SERIAL"):
        serials = [x for x in serials if x == os.getenv("SERIAL")]

      pandas = [Panda(x, claim=claim) for x in serials]
      decoders = [codecs.getincrementaldecoder('utf-8')() for _ in pandas]

      if not len(pandas):
        print("no pandas found")
        if no_reconnect:
          sys.exit(0)
        time.sleep(1)
        continue

      if os.getenv("BAUD") is not None:
        for panda in pandas:
          panda.set_uart_baud(port_number, int(os.getenv("BAUD")))  # type: ignore

      while True:
        for i, panda in enumerate(pandas):
          while True:
            ret = panda.serial_read(port_number)
            if len(ret) > 0:
              decoded = decoders[i].decode(ret)
              if no_color:
                sys.stdout.write(decoded)
              else:
                sys.stdout.write(setcolor[i] + decoded + unsetcolor)
              sys.stdout.flush()
            else:
              break
          if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            ln = sys.stdin.readline()
            if ln == "h\n":
              h = panda.health()
              print(h)
            elif ln == "r\n":
              panda.reset()
            elif ln == "enable_iwdg\n":
              panda.flash_config_write_SYS(0, 0, 0, 0, 0, 0, 0, 0)
            elif ln == "disable_iwdg\n":
              panda.flash_config_write_CAN(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            elif ln == "flashwrite0\n":
              panda.flash_config_write_CAN(1, 37, 0, 15, 8, 1, 48, 12, 0, 1)
            elif ln == "flashwrite1\n":
              panda.flash_config_write_CAN(2, 37, 0, 1 , 8, 2, 28, 4, 0, 1)
            elif ln == "flashwrite2\n":
              panda.flash_config_write_ADC(11, 0, 0, 100, 0, 1)
            elif ln == "flashwrite3\n":
              panda.flash_config_write_ADC(12, 0x400, 0, 100, 0, 1)
            elif ln == "flashwrite4\n":
              panda.flash_config_write_ADC(13, 0x500, 0, 100, 0, 1)
            elif ln == "flashwrite5\n":
              panda.flash_config_write_ADC(14, 0x570, 0, 100, 0, 1)
            elif ln == "flashread\n":
              ret = panda.flash_config_read()
              for r in ret:
                print(r)
            elif ln == "flashwipe\n":
              panda.flash_wipe_config()
            if claim:
              panda.serial_write(port_number, ln)
          time.sleep(0.01)
    except KeyboardInterrupt:
      break
    except Exception:
      print("panda disconnected!")
      time.sleep(0.5)
      