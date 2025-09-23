# python library to interface with panda
import datetime
import struct
import hashlib
import socket
import usb1
import os
import time
import traceback
import sys
from .dfu import PandaDFU  # pylint: disable=import-error
from .flash_release import flash_release  # noqa pylint: disable=import-error
from .update import ensure_st_up_to_date  # noqa pylint: disable=import-error
from .serial import PandaSerial  # noqa pylint: disable=import-error
from .isotp import isotp_send, isotp_recv  # pylint: disable=import-error


__version__ = '0.0.9'

BASEDIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../")
DEFAULT_FW_FN = os.path.join(BASEDIR, "board", "obj", "panda.bin.signed")

DEBUG = os.getenv("PANDADEBUG") is not None

def parse_can_buffer(dat):
  ret = []
  for j in range(0, len(dat), 0x10):
    ddat = dat[j:j + 0x10]
    f1, f2 = struct.unpack("II", ddat[0:8])
    extended = 4
    if f1 & extended:
      address = f1 >> 3
    else:
      address = f1 >> 21
    dddat = ddat[8:8 + (f2 & 0xF)]
    if DEBUG:
      print(f"  R 0x{address:x}: 0x{dddat.hex()}")
    ret.append((address, f2 >> 16, dddat, (f2 >> 4) & 0xFF))
  return ret

class PandaWifiStreaming(object):
  def __init__(self, ip="192.168.0.10", port=1338):
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.setblocking(0)
    self.ip = ip
    self.port = port
    self.kick()

  def kick(self):
    # must be called at least every 5 seconds
    self.sock.sendto("hello", (self.ip, self.port))

  def can_recv(self):
    ret = []
    while True:
      try:
        dat, addr = self.sock.recvfrom(0x200 * 0x10)
        if addr == (self.ip, self.port):
          ret += parse_can_buffer(dat)
      except socket.error as e:
        if e.errno != 35 and e.errno != 11:
          traceback.print_exc()
        break
    return ret

# stupid tunneling of USB over wifi and SPI
class WifiHandle(object):
  def __init__(self, ip="192.168.0.10", port=1337):
    self.sock = socket.create_connection((ip, port))

  def __recv(self):
    ret = self.sock.recv(0x44)
    length = struct.unpack("I", ret[0:4])[0]
    return ret[4:4 + length]

  def controlWrite(self, request_type, request, value, index, data, timeout=0):
    # ignore data in reply, panda doesn't use it
    return self.controlRead(request_type, request, value, index, 0, timeout)

  def controlRead(self, request_type, request, value, index, length, timeout=0):
    self.sock.send(struct.pack("HHBBHHH", 0, 0, request_type, request, value, index, length))
    return self.__recv()

  def bulkWrite(self, endpoint, data, timeout=0):
    if len(data) > 0x10:
      raise ValueError("Data must not be longer than 0x10")
    self.sock.send(struct.pack("HH", endpoint, len(data)) + data)
    self.__recv()  # to /dev/null

  def bulkRead(self, endpoint, length, timeout=0):
    self.sock.send(struct.pack("HH", endpoint, 0))
    return self.__recv()

  def close(self):
    self.sock.close()

# *** normal mode ***

class Panda(object):

  # matches cereal.car.CarParams.SafetyModel
  SAFETY_SILENT = 0
  SAFETY_HONDA_NIDEC = 1
  SAFETY_TOYOTA = 2
  SAFETY_ELM327 = 3
  SAFETY_GM = 4
  SAFETY_HONDA_BOSCH_GIRAFFE = 5
  SAFETY_FORD = 6
  SAFETY_HYUNDAI = 8
  SAFETY_CHRYSLER = 9
  SAFETY_TESLA = 10
  SAFETY_SUBARU = 11
  SAFETY_MAZDA = 13
  SAFETY_NISSAN = 14
  SAFETY_VOLKSWAGEN_MQB = 15
  SAFETY_ALLOUTPUT = 17
  SAFETY_GM_ASCM = 18
  SAFETY_NOOUTPUT = 19
  SAFETY_HONDA_BOSCH_HARNESS = 20
  SAFETY_VOLKSWAGEN_PQ = 21
  SAFETY_SUBARU_LEGACY = 22
  SAFETY_HYUNDAI_LEGACY = 23

  SERIAL_DEBUG = 0
  SERIAL_ESP = 1
  SERIAL_LIN1 = 2
  SERIAL_LIN2 = 3

  GMLAN_CAN2 = 1
  GMLAN_CAN3 = 2

  REQUEST_IN = usb1.ENDPOINT_IN | usb1.TYPE_VENDOR | usb1.RECIPIENT_DEVICE
  REQUEST_OUT = usb1.ENDPOINT_OUT | usb1.TYPE_VENDOR | usb1.RECIPIENT_DEVICE

  HW_TYPE_UNKNOWN = b'\x00'
  HW_TYPE_WHITE_PANDA = b'\x01'
  HW_TYPE_GREY_PANDA = b'\x02'
  HW_TYPE_BLACK_PANDA = b'\x03'
  HW_TYPE_PEDAL = b'\x04'
  HW_TYPE_UNO = b'\x05'

  CLOCK_SOURCE_MODE_DISABLED = 0
  CLOCK_SOURCE_MODE_FREE_RUNNING = 1
  CLOCK_SOURCE_MODE_EXTERNAL_SYNC = 2

  def __init__(self, serial=None, claim=True):
    self._serial = serial
    self._handle = None
    self.connect(claim)

  def close(self):
    self._handle.close()
    self._handle = None

  def connect(self, claim=True, wait=False):
    if self._handle is not None:
      self.close()

    if self._serial == "WIFI":
      self._handle = WifiHandle()
      print("opening WIFI device")
      self.wifi = True
    else:
      context = usb1.USBContext()
      self._handle = None
      self.wifi = False

      while 1:
        try:
          for device in context.getDeviceList(skip_on_error=True):
            if device.getVendorID() == 0xbbaa and device.getProductID() in [0xddcc, 0xddee]:
              try:
                this_serial = device.getSerialNumber()
              except Exception:
                continue
              if self._serial is None or this_serial == self._serial:
                self._serial = this_serial
                print("opening device", self._serial, hex(device.getProductID()))
                self.bootstub = device.getProductID() == 0xddee
                self.legacy = (device.getbcdDevice() != 0x2300)
                self._handle = device.open()
                if sys.platform not in ["win32", "cygwin", "msys", "darwin"]:
                  self._handle.setAutoDetachKernelDriver(True)
                if claim:
                  self._handle.claimInterface(0)
                  # self._handle.setInterfaceAltSetting(0, 0)  # Issue in USB stack
                break
        except Exception as e:
          print("exception", e)
          traceback.print_exc()
        if not wait or self._handle is not None:
          break
        context = usb1.USBContext()  # New context needed so new devices show up
    assert(self._handle is not None)
    print("connected")

  def reset(self, enter_bootstub=False, enter_bootloader=False):
    # reset
    try:
      if enter_bootloader:
        self._handle.controlWrite(Panda.REQUEST_IN, 0xd1, 0, 0, b'')
      else:
        if enter_bootstub:
          self._handle.controlWrite(Panda.REQUEST_IN, 0xd1, 1, 0, b'')
        else:
          self._handle.controlWrite(Panda.REQUEST_IN, 0xd8, 0, 0, b'')
    except Exception:
      pass
    if not enter_bootloader:
      self.reconnect()

  def reconnect(self):
    self.close()
    time.sleep(1.0)
    success = False
    # wait up to 15 seconds
    for i in range(0, 15):
      try:
        self.connect()
        success = True
        break
      except Exception:
        print("reconnecting is taking %d seconds..." % (i + 1))
        try:
          dfu = PandaDFU(PandaDFU.st_serial_to_dfu_serial(self._serial))
          dfu.recover()
        except Exception:
          pass
        time.sleep(1.0)
    if not success:
      raise Exception("reconnect failed")

  @staticmethod
  def flash_static(handle, code):
    # confirm flasher is present
    fr = handle.controlRead(Panda.REQUEST_IN, 0xb0, 0, 0, 0xc)
    assert fr[4:8] == b"\xde\xad\xd0\x0d"

    # unlock flash
    print("flash: unlocking")
    handle.controlWrite(Panda.REQUEST_IN, 0xb1, 0, 0, b'')

    # erase sectors 1 through 3
    print("flash: erasing")
    for i in range(1, 4):
      handle.controlWrite(Panda.REQUEST_IN, 0xb2, i, 0, b'')

    # flash over EP2
    STEP = 0x10
    print("flash: flashing")
    for i in range(0, len(code), STEP):
      handle.bulkWrite(2, code[i:i + STEP])

    # reset
    print("flash: resetting")
    try:
      handle.controlWrite(Panda.REQUEST_IN, 0xd8, 0, 0, b'')
    except Exception:
      pass

  def flash(self, fn=DEFAULT_FW_FN, code=None, reconnect=True):
    print("flash: main version is " + self.get_version())
    if not self.bootstub:
      self.reset(enter_bootstub=True)
    assert(self.bootstub)

    if code is None:
      with open(fn, "rb") as f:
        code = f.read()

    # get version
    print("flash: bootstub version is " + self.get_version())

    # do flash
    Panda.flash_static(self._handle, code)

    # reconnect
    if reconnect:
      self.reconnect()

  def recover(self, timeout=None):
    self.reset(enter_bootstub=True)
    self.reset(enter_bootloader=True)
    t_start = time.time()
    while len(PandaDFU.list()) == 0:
      print("waiting for DFU...")
      time.sleep(0.1)
      if timeout is not None and (time.time() - t_start) > timeout:
        return False

    dfu = PandaDFU(PandaDFU.st_serial_to_dfu_serial(self._serial))
    dfu.recover()

    # reflash after recover
    self.connect(True, True)
    self.flash()
    return True

  @staticmethod
  def flash_ota_st():
    ret = os.system("cd %s && make clean && make ota" % (os.path.join(BASEDIR, "board")))
    time.sleep(1)
    return ret == 0

  @staticmethod
  def flash_ota_wifi(release=False):
    release_str = "RELEASE=1" if release else ""
    ret = os.system("cd {} && make clean && {} make ota".format(os.path.join(BASEDIR, "boardesp"), release_str))
    time.sleep(1)
    return ret == 0

  @staticmethod
  def list():
    context = usb1.USBContext()
    ret = []
    try:
      for device in context.getDeviceList(skip_on_error=True):
        if device.getVendorID() == 0xbbaa and device.getProductID() in [0xddcc, 0xddee]:
          try:
            ret.append(device.getSerialNumber())
          except Exception:
            continue
    except Exception:
      pass
    # TODO: detect if this is real
    # ret += ["WIFI"]
    return ret

  def call_control_api(self, msg):
    self._handle.controlWrite(Panda.REQUEST_OUT, msg, 0, 0, b'')

  # ******************* health *******************

  def health(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xd2, 0, 0, 43)
    a = struct.unpack("<IIIIIIIIBBBBBBBHBB", dat)
    return {
      "uptime": a[0],
      "voltage": a[1],
      "current": a[2],
      "can_rx_errs": a[3],
      "can_send_errs": a[4],
      "can_fwd_errs": a[5],
      "gmlan_send_errs": a[6],
      "faults": a[7],
      "ignition_line": a[8],
      "ignition_can": a[9],
      "controls_allowed": a[10],
      "gas_interceptor_detected": a[11],
      "car_harness_status": a[12],
      "usb_power_mode": a[13],
      "safety_mode": a[14],
      "safety_param": a[15],
      "fault_status": a[16],
      "power_save_enabled": a[17]
    }

  # ******************* control *******************

  def enter_bootloader(self):
    try:
      self._handle.controlWrite(Panda.REQUEST_OUT, 0xd1, 0, 0, b'')
    except Exception as e:
      print(e)

  def get_version(self):
    return self._handle.controlRead(Panda.REQUEST_IN, 0xd6, 0, 0, 0x40).decode('utf8')

  @staticmethod
  def get_signature_from_firmware(fn):
    f = open(fn, 'rb')
    f.seek(-128, 2)  # Seek from end of file
    return f.read(128)

  def get_signature(self):
    part_1 = self._handle.controlRead(Panda.REQUEST_IN, 0xd3, 0, 0, 0x40)
    part_2 = self._handle.controlRead(Panda.REQUEST_IN, 0xd4, 0, 0, 0x40)
    return bytes(part_1 + part_2)

  def get_type(self):
    return self._handle.controlRead(Panda.REQUEST_IN, 0xc1, 0, 0, 0x40)

  def is_white(self):
    return self.get_type() == Panda.HW_TYPE_WHITE_PANDA

  def is_grey(self):
    return self.get_type() == Panda.HW_TYPE_GREY_PANDA

  def is_black(self):
    return self.get_type() == Panda.HW_TYPE_BLACK_PANDA

  def is_uno(self):
    return self.get_type() == Panda.HW_TYPE_UNO

  def has_obd(self):
    return (self.is_uno() or self.is_black())

  def get_serial(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xd0, 0, 0, 0x20)
    hashsig, calc_hash = dat[0x1c:], hashlib.sha1(dat[0:0x1c]).digest()[0:4]
    assert(hashsig == calc_hash)
    return [dat[0:0x10].decode("utf8"), dat[0x10:0x10 + 10].decode("utf8")]

  def get_secret(self):
    return self._handle.controlRead(Panda.REQUEST_IN, 0xd0, 1, 0, 0x10)

  # ******************* configuration *******************

  def set_usb_power(self, on):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe6, int(on), 0, b'')

  def set_power_save(self, power_save_enabled=0):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe7, int(power_save_enabled), 0, b'')

  def set_esp_power(self, on):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xd9, int(on), 0, b'')

  def esp_reset(self, bootmode=0):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xda, int(bootmode), 0, b'')
    time.sleep(0.2)

  def set_safety_mode(self, mode=SAFETY_SILENT):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xdc, mode, 0, b'')

  def set_can_forwarding(self, from_bus, to_bus):
    # TODO: This feature may not work correctly with saturated buses
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xdd, from_bus, to_bus, b'')

  def set_gmlan(self, bus=2):
    # TODO: check panda type
    if bus is None:
      self._handle.controlWrite(Panda.REQUEST_OUT, 0xdb, 0, 0, b'')
    elif bus in [Panda.GMLAN_CAN2, Panda.GMLAN_CAN3]:
      self._handle.controlWrite(Panda.REQUEST_OUT, 0xdb, 1, bus, b'')

  def set_obd(self, obd):
    # TODO: check panda type
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xdb, int(obd), 0, b'')

  def set_can_loopback(self, enable):
    # set can loopback mode for all buses
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe5, int(enable), 0, b'')

  def set_can_enable(self, bus_num, enable):
    # sets the can transceiver enable pin
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf4, int(bus_num), int(enable), b'')

  def set_can_speed_kbps(self, bus, speed):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xde, bus, int(speed * 10), b'')

  def set_uart_baud(self, uart, rate):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe4, uart, int(rate / 300), b'')

  def set_uart_parity(self, uart, parity):
    # parity, 0=off, 1=even, 2=odd
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe2, uart, parity, b'')

  def set_uart_callback(self, uart, install):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe3, uart, int(install), b'')

  # ******************* can *******************

  # The panda will NAK CAN writes when there is CAN congestion.
  # libusb will try to send it again, with a max timeout.
  # Timeout is in ms. If set to 0, the timeout is infinite.
  CAN_SEND_TIMEOUT_MS = 10

  def can_send_many(self, arr, timeout=CAN_SEND_TIMEOUT_MS):
    snds = []
    transmit = 1
    extended = 4
    for addr, _, dat, bus in arr:
      assert len(dat) <= 8
      if DEBUG:
        print(f"  W 0x{addr:x}: 0x{dat.hex()}")
      if addr >= 0x800:
        rir = (addr << 3) | transmit | extended
      else:
        rir = (addr << 21) | transmit
      snd = struct.pack("II", rir, len(dat) | (bus << 4)) + dat
      snd = snd.ljust(0x10, b'\x00')
      snds.append(snd)

    while True:
      try:
        if self.wifi:
          for s in snds:
            self._handle.bulkWrite(3, s)
        else:
          self._handle.bulkWrite(3, b''.join(snds), timeout=timeout)
        break
      except (usb1.USBErrorIO, usb1.USBErrorOverflow):
        print("CAN: BAD SEND MANY, RETRYING")

  def can_send(self, addr, dat, bus, timeout=CAN_SEND_TIMEOUT_MS):
    self.can_send_many([[addr, None, dat, bus]], timeout=timeout)

  def can_recv(self):
    dat = bytearray()
    while True:
      try:
        dat = self._handle.bulkRead(1, 0x10 * 256)
        break
      except (usb1.USBErrorIO, usb1.USBErrorOverflow):
        print("CAN: BAD RECV, RETRYING")
        time.sleep(0.1)
    return parse_can_buffer(dat)

  def can_clear(self, bus):
    """Clears all messages from the specified internal CAN ringbuffer as
    though it were drained.

    Args:
      bus (int): can bus number to clear a tx queue, or 0xFFFF to clear the
        global can rx queue.

    """
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf1, bus, 0, b'')

  # ******************* isotp *******************

  def isotp_send(self, addr, dat, bus, recvaddr=None, subaddr=None):
    return isotp_send(self, dat, addr, bus, recvaddr, subaddr)

  def isotp_recv(self, addr, bus=0, sendaddr=None, subaddr=None):
    return isotp_recv(self, addr, bus, sendaddr, subaddr)

  # ******************* serial *******************

  def serial_read(self, port_number):
    ret = []
    while 1:
      lret = bytes(self._handle.controlRead(Panda.REQUEST_IN, 0xe0, port_number, 0, 0x40))
      if len(lret) == 0:
        break
      ret.append(lret)
    return b''.join(ret)

  def serial_write(self, port_number, ln):
    ret = 0
    for i in range(0, len(ln), 0x20):
      ret += self._handle.bulkWrite(2, struct.pack("B", port_number) + ln[i:i + 0x20])
    return ret

  def serial_clear(self, port_number):
    """Clears all messages (tx and rx) from the specified internal uart
    ringbuffer as though it were drained.

    Args:
      port_number (int): port number of the uart to clear.

    """
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf2, port_number, 0, b'')

  # ******************* kline *******************

  # pulse low for wakeup
  def kline_wakeup(self, k=True, l=True):
    assert k or l, "must specify k-line, l-line, or both"
    if DEBUG:
      print("kline wakeup...")
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf0, 2 if k and l else int(l), 0, b'')
    if DEBUG:
      print("kline wakeup done")

  def kline_5baud(self, addr, k=True, l=True):
    assert k or l, "must specify k-line, l-line, or both"
    if DEBUG:
      print("kline 5 baud...")
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf4, 2 if k and l else int(l), addr, b'')
    if DEBUG:
      print("kline 5 baud done")

  def kline_drain(self, bus=2):
    # drain buffer
    bret = bytearray()
    while True:
      ret = self._handle.controlRead(Panda.REQUEST_IN, 0xe0, bus, 0, 0x40)
      if len(ret) == 0:
        break
      elif DEBUG:
        print(f"kline drain: 0x{ret.hex()}")
      bret += ret
    return bytes(bret)

  def kline_ll_recv(self, cnt, bus=2):
    echo = bytearray()
    while len(echo) != cnt:
      ret = self._handle.controlRead(Panda.REQUEST_OUT, 0xe0, bus, 0, cnt - len(echo))
      if DEBUG and len(ret) > 0:
        print(f"kline recv: 0x{ret.hex()}")
      echo += ret
    return bytes(echo)

  def kline_send(self, x, bus=2, checksum=True):
    self.kline_drain(bus=bus)
    if checksum:
      x += bytes([sum(x) % 0x100])
    for i in range(0, len(x), 0xf):
      ts = x[i:i + 0xf]
      if DEBUG:
        print(f"kline send: 0x{ts.hex()}")
      self._handle.bulkWrite(2, bytes([bus]) + ts)
      echo = self.kline_ll_recv(len(ts), bus=bus)
      if echo != ts:
        print(f"**** ECHO ERROR {i} ****")
        print(f"0x{echo.hex()}")
        print(f"0x{ts.hex()}")
    assert echo == ts

  def kline_recv(self, bus=2, header_len=4):
    # read header (last byte is length)
    msg = self.kline_ll_recv(header_len, bus=bus)
    # read data (add one byte to length for checksum)
    msg += self.kline_ll_recv(msg[-1]+1, bus=bus)
    return msg

  def send_heartbeat(self):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf3, 0, 0, b'')

  # ******************* RTC *******************
  def set_datetime(self, dt):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xa1, int(dt.year), 0, b'')
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xa2, int(dt.month), 0, b'')
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xa3, int(dt.day), 0, b'')
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xa4, int(dt.isoweekday()), 0, b'')
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xa5, int(dt.hour), 0, b'')
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xa6, int(dt.minute), 0, b'')
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xa7, int(dt.second), 0, b'')

  def get_datetime(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xa0, 0, 0, 8)
    a = struct.unpack("HBBBBBB", dat)
    return datetime.datetime(a[0], a[1], a[2], a[4], a[5], a[6])

  # ******************* IR *******************
  def set_ir_power(self, percentage):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xb0, int(percentage), 0, b'')

  # ******************* Fan ******************
  def set_fan_power(self, percentage):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xb1, int(percentage), 0, b'')

  def get_fan_rpm(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xb2, 0, 0, 2)
    a = struct.unpack("H", dat)
    return a[0]

  # ****************** Phone *****************
  def set_phone_power(self, enabled):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xb3, int(enabled), 0, b'')

  # ************** Clock Source **************
  def set_clock_source_mode(self, mode):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf5, int(mode), 0, b'')

  # ****************** Siren *****************
  def set_siren(self, enabled):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf6, int(enabled), 0, b'')

  # ****************** Debug *****************
  def set_green_led(self, enabled):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf7, int(enabled), 0, b'')

  # # ************ set relay chimera ***********
  # def set_relay_pwr(self, enabled):
  #   self._handle.controlWrite(Panda.REQUEST_OUT, 0xb4, int(enabled), 0, b'')

  # # ************ set relay chimera ***********
  # def set_ign_obdc(self, enabled):
  #   self._handle.controlWrite(Panda.REQUEST_OUT, 0xb5, int(enabled), 0, b'')

  def flash_wipe_config(self):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xFD, 0, 0, b'')

  def flash_config_write_SYS(self, debug_lvl, can_out_en, iwdg_en):
    cfg_extra = bytes(20)  # expanded from 13 to 20
    cfg_type = 1  # CFG_TYPE_SYS

    dat = struct.pack("<3B20sB",
                      debug_lvl,
                      can_out_en,
                      iwdg_en,
                      cfg_extra,
                      cfg_type)
    print(f"Data length: {len(dat)}")
    print(f"Data (hex): {dat.hex()}")
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xFE, 0, 0, dat)

  def flash_config_write_CAN(self, index, can_id, scale_offs, scale_mult, msg_len_bytes, sig_type, shift_amt, sig_len, endian_type, enabled, is_signed):
    # assert 0 <= index < 32
    extra = bytes(6)
    cfg_type = 2  # CFG_TYPE_CAN

    dat = struct.pack("<Iih7B6sB",
                      can_id,
                      scale_offs,
                      scale_mult,
                      msg_len_bytes,
                      sig_type,
                      shift_amt,
                      sig_len,
                      endian_type,
                      enabled,
                      is_signed,
                      extra,
                      cfg_type)
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xFE, index, 0, dat)

  def flash_config_write_ADC(self, index, adc1, adc2, adc_tolerance, adc_num, adc_en):
    # assert 11 <= index < 16
    adc_reserved = bytes(4)
    extra = bytes(7)
    cfg_type = 3  # CFG_TYPE_ADC

    dat = struct.pack("<IIHB4sB7sB",
                      adc1,
                      adc2,
                      adc_tolerance,
                      adc_num,
                      adc_reserved,
                      adc_en,
                      extra,
                      cfg_type)
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xFE, index, 0, dat)

  def flash_config_write_VSS(self, index, vss_ppd, is_kmh):
    cfg_extra = bytes(20) 
    cfg_type = 4  # CFG_TYPE_VSS

    dat = struct.pack("<HB20sB",
                      vss_ppd,
                      is_kmh,
                      cfg_extra,
                      cfg_type)
    print(f"Data length: {len(dat)}")
    print(f"Data (hex): {dat.hex()}")
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xFE, index, 0, dat)

  def flash_config_write_RELAY(self, index, label, gpio_en, gpio_in, can_addr, sig_len, shift_amt, can_cmp):
    cfg_extra = bytes(10)
    cfg_type = 5 #CFG_TYPE_RELAY

    relay_labels = {
      "TURN_L_FRONT": 0,
      "DRL_L": 1,
      "HEAD_L": 2,
      "BRIGHT_L": 3,
      "FOG_L_FRONT": 4,
      "TURN_R_FRONT": 5,
      "DRL_R": 6,
      "HEAD_R": 7,
      "BRIGHT_R": 8,
      "FOG_R_FRONT": 9,
      "TURN_L_REAR": 10,
      "BRAKE_L": 11,
      "FOG_L_REAR": 12,
      "REVERSE_L": 13,
      "SIDE_L": 14,
      "TURN_R_REAR": 15,
      "BRAKE_R": 16,
      "FOG_R_REAR": 17,
      "REVERSE_R": 18,
      "SIDE_R": 19,
      "BRAKE_CENTER": 20,
      "SP_1": 21,
      "SP_2": 22,
      "SP_3": 23,
      "SP_4": 24,
      "SP_5": 25,
      "SP_6": 26,
      "SP_7": 27,
      "SP_8": 28,
      "SP_9": 29,
      "SP_10": 30,
      "SP_11": 31,
    }

    type = 1 << relay_labels[label.upper()] if label.upper() in relay_labels else 0

    dat = struct.pack("<II5B10sB",
                      type,
                      can_cmp,
                      gpio_en,
                      gpio_in,
                      can_addr,
                      sig_len,
                      shift_amt,
                      cfg_extra,
                      cfg_type)
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xFE, index, 0, dat)

  def flash_config_read(self):
    MAX_CONFIG_ENTRIES = 32
    ENTRY_SIZE = 24
    HEADER_SIZE = 4  # magic
    CRC_SIZE = 4
    TOTAL_SIZE = HEADER_SIZE + ENTRY_SIZE * MAX_CONFIG_ENTRIES + CRC_SIZE
    CHUNK_SIZE = 64

    raw = bytearray()
    for offset in range(0, TOTAL_SIZE, CHUNK_SIZE):
      chunk = self._handle.controlRead(Panda.REQUEST_IN, 0xFF, offset, 0, CHUNK_SIZE)
      raw += chunk

    magic, = struct.unpack_from("<I", raw, 0)
    if magic != 0xDEADBEEF:
      raise ValueError("Invalid config magic")

    entries = []
    for i in range(MAX_CONFIG_ENTRIES):
      offset = HEADER_SIZE + i * ENTRY_SIZE
      entry_bytes = raw[offset:offset + ENTRY_SIZE]
      if len(entry_bytes) != ENTRY_SIZE:
        print(f"Entry {i}: incomplete read")
        continue

      cfg_type = entry_bytes[-1]  # last byte

      if cfg_type == 1:  # SYSTEM
        debug_lvl, can_out_en, iwdg_en, cfg_extra, flags, cfg_type = struct.unpack_from("<3B20s7sB", raw, offset)
        entries.append({
          "index": i,
          "cfg_type": "SYSTEM",
          "debug_lvl": debug_lvl,
          "can_out_en": can_out_en,
          "iwdg_en": iwdg_en,
          "cfg_extra": cfg_extra,
          "flags": flags,
        })

      elif cfg_type == 2:  # CAN
        can_id, scale_offs, scale_mult, msg_len, sig_type, shift_amt, sig_len, endian, enabled, is_signed, raw_flags, cfg_type = struct.unpack_from("<Iih7B6sB", raw, offset)
        entries.append({
          "index": i,
          "cfg_type": "CAN",
          "can_id": can_id,
          "scale_offs": scale_offs,
          "scale_mult": scale_mult,
          "msg_len_bytes": msg_len,
          "sig_type": sig_type,
          "shift_amt": shift_amt,
          "sig_len": sig_len,
          "endian_type": endian,
          "enabled": enabled,
          "is_signed": is_signed,
          "flags": raw_flags,
        })

      elif cfg_type == 3:  # ADC
        adc1, adc2, adc_tolerance, adc_num, adc_reserved, adc_en, raw_flags, cfg_type = struct.unpack_from("<IIHB4sB7sB", raw, offset)
        entries.append({
          "index": i,
          "cfg_type": "ADC",
          "adc1": adc1,
          "adc2": adc2,
          "adc_tolerance": adc_tolerance,
          "adc_num": adc_num,
          "adc_reserved": adc_reserved,
          "adc_en": adc_en,
          "flags": raw_flags,
        })

      elif cfg_type == 4:
        vss_ppd, is_kmh, extra, cfg_type = struct.unpack_from("<HB20sB", raw, offset)
        entries.append({ 
          "index": i,
          "cfg_type": "VSS",
          "vss_ppd": vss_ppd,
          "is_kmh": is_kmh,
          "extra": extra,
        })
      elif cfg_type == 5:  # RELAY
        type_val, can_cmp_val, gpio_en, gpio_in, can_addr, sig_len, shift_amt, extra, cfg_type = struct.unpack_from("<II5B10sB", raw, offset)
        entries.append({
          "index": i,
          "cfg_type": "RELAY",
          "label": type_val,
          "can_cmp_val": can_cmp_val,
          "gpio_en": gpio_en,
          "gpio_in": gpio_in,
          "can_addr": can_addr,
          "sig_len": sig_len,
          "shift_amt": shift_amt,
          "extra": extra,
        })

    return entries
  