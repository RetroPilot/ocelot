#!/usr/bin/env python3

import os
import sys
import usb1
from firmware.python import Panda

RED = '\033[31m'
GREEN = '\033[32m'

CHIMERA_PANDA_VID = 0xbbaa
CHIMERA_PANDA_PIDS = [0xddcc, 0xddee]  # Panda + Chimera variants

SUPPORTED_PRODUCTS = ["Chimera", "Actuator Core", "Relay Core"]

#TODO: handle and format unformatted flash

# Unified configuration list (index, label, supported types)
chimera_config_list = [
  (0,  "System Config",           ["SYS"]),
  (1,  "Steer Angle Major",       ["CAN"]),
  (2,  "Steer Angle Minor",       ["CAN"]),
  (3,  "Steer Angle Rate",        ["CAN"]),
  (4,  "Vehicle Speed",           ["CAN", "VSS"]),
  (5,  "Ignition",                ["CAN"]),
  (6,  "Engine RPM",              ["CAN"]),
  (7,  "Brake Pressed",           ["CAN"]),
  (11, "Cruise Button Cancel",    ["CAN", "ADC"]),
  (12, "Cruise Button Set/Down",  ["CAN", "ADC"]),
  (13, "Cruise Button Res/Up",    ["CAN", "ADC"]),
  (14, "Cruise Button On/Off",    ["CAN", "ADC"]),
]

actuator_core_config_list = [
  (0,  "System Config",           ["SYS"]),
  (1,  "TPS Config",              ["ADC"]),
]

relay_core_config_list = [
  (0, "System Config",            ["SYS"]),
  (1, "Relay 1 Config",           ["RELAY"]),
  (2, "Relay 2 Config",           ["RELAY"]),
  (3, "Relay 3 Config",           ["RELAY"]),
  (4, "Relay 4 Config",           ["RELAY"]),
  (5, "Relay 5 Config",           ["RELAY"]),
  (6, "Relay 6 Config",           ["RELAY"]),
  (7, "Relay 7 Config",           ["RELAY"]),
  (8, "Relay 8 Config",           ["RELAY"]),
]

relay_core_config_tags = [
  "TURN_L_FRONT",
  "DRL_L",
  "HEAD_L",
  "BRIGHT_L",
  "FOG_L_FRONT",
  "TURN_R_FRONT",
  "DRL_R",
  "HEAD_R",
  "BRIGHT_R",
  "FOG_R_FRONT",
  "TURN_L_REAR",
  "BRAKE_L",
  "FOG_L_REAR",
  "REVERSE_L",
  "TAIL_L",
  "TURN_R_REAR",
  "BRAKE_R",
  "FOG_R_REAR",
  "REVERSE_R",
  "TAIL_R",
  "BRAKE_CENTER",
  "SP_1",
  "SP_2",
  "SP_3",
  "SP_4",
  "SP_5",
  "SP_6",
  "SP_7",
  "SP_8",
  "SP_9",
  "SP_10",
]

def get_relay_label_from_type(type_val):
  for i, label in enumerate(relay_core_config_tags):
    if type_val & (1 << i):
      return label
  return "UNKNOWN"

def list_devices():
  context = usb1.USBContext()
  results = []

  for device in context.getDeviceList(skip_on_error=True):
    if device.getVendorID() == CHIMERA_PANDA_VID and device.getProductID() in CHIMERA_PANDA_PIDS:
      try:
        handle = device.open()
        serial = handle.getSerialNumber()
        manufacturer = handle.getManufacturer()
        product = handle.getProduct()
        if manufacturer != "commaai" and any(p.lower() in product.lower() for p in SUPPORTED_PRODUCTS):
          results.append({
            "product": product,
            "serial": serial
          })
        handle.close()
      except usb1.USBErrorAccess:
        print(f"{RED}Permission denied for device on bus {device.getBusNumber()}:{device.getDeviceAddress()}")
      except Exception as e:
        print(f"Error accessing device: {e}")
  return results

def print_config_entries(entries):
  for e in entries:
    idx = e.get("index", "N/A")
    cfg_type = e.get("cfg_type", "N/A")

    if cfg_type == "CAN":
      print(f"[{idx}] CAN: ID=0x{e['can_id']:X}, sig_type={e['sig_type']}, shift={e['shift_amt']}, len={e['sig_len']} bits, endian={e['endian_type']}, mult={e['scale_mult']}, offs={e['scale_offs']}, enabled={e['enabled']} is_signed={e['is_signed']}")
    elif cfg_type == "ADC":
      print(f"[{idx}] ADC: adc1={e['adc1']}, adc2={e['adc2']}, tol={e['adc_tolerance']}, chan={e['adc_num']}, en={e['adc_en']}")
    elif cfg_type == "SYSTEM":
      # print(f"[{idx}] SYSTEM: debug_lvl={e['debug_lvl']}, can_out={e['can_out_en']}, iwdg_en={e['iwdg_en']}")
      print("SYS_PARAMS: ", e)
    elif cfg_type == "VSS":
      print("VSS: ", e)
    elif cfg_type == "RELAY":
      label = get_relay_label_from_type(e['label'])
      print(f"[{idx}] RELAY: {label}, can_cmp={e['can_cmp_val']}, gpio_en={e['gpio_en']}, gpio_in={e['gpio_in']}, can_addr=0x{e['can_addr']:X}, sig_len={e['sig_len']}, shift={e['shift_amt']}")
    else:
      print(f"[{idx}] UNKNOWN or DISABLED")

if __name__ == "__main__":
  while True:
    devices = list_devices()

    if not devices:
      print(f"{RED}No supported devices found.")
      sys.exit(1)

    print(f"{GREEN}Please select a device to configure:")
    for idx, device in enumerate(devices):
      product = device.get("product", "N/A")
      serial = device.get("serial", "N/A")
      print(f"{idx}: Product: {product}, Serial: {serial}")

    selected = None
    while True:
      ln = input("Enter the number of the device to select: ").strip()
      if ln.isdigit():
        i = int(ln)
        if 0 <= i < len(devices):
          selected = devices[i]
          break
      print(f"{RED}Invalid input. Please enter a valid number from the list.")

    print(f"{GREEN}Selected device: {selected}")

    active_config_list = None
    selected_product_name = selected.get("product", "").upper()

    if "CHIMERA" in selected_product_name:
        active_config_list = chimera_config_list
        print(f"{GREEN}Chimera device detected. Using Chimera config.")
    elif "ACTUATOR CORE" in selected_product_name:
        active_config_list = actuator_core_config_list
        print(f"{GREEN}Actuator Core device detected. Using Actuator Core config.")
    elif "RELAY CORE" in selected_product_name:
        active_config_list = relay_core_config_list
        print(f"{GREEN}Actuator Core device detected. Using Actuator Core config.")
    else:
        print(f"{RED}Unknown device type. Cannot determine configuration options.")
        sys.exit(1)


    device_serial = selected.get("serial")
    print(f"Attempting to connect to device with serial: {device_serial}")
    try:
      panda = Panda(serial=device_serial)
      print(f"{GREEN}Connected. Reading existing flash config:")
      try:
        flash_entries = panda.flash_config_read()
        print_config_entries(flash_entries)
      except Exception as e:
        print(f"{RED}Failed to read flash config: {e}")
        if str(e) == "Invalid config magic":
          print(f"{GREEN}Would you like to format this device?")
          ln = input("Y/N").strip()
          if ln == ("Y"):
            panda.flash_wipe_config()
            print("FLASH WIPED")
            panda.reset()
            continue
          if ln == ("N"):
            break

      print("\nAvailable configuration options:")

      for idx, (flash_idx, label, types) in enumerate(active_config_list):
        print(f"{idx}: {label} (Flash Index {flash_idx}) - Types: {', '.join(types)}")

      while True:
        ln = input("Enter the number of the configuration option to select: ").strip()
        if ln.isdigit():
          i = int(ln)

          if 0 <= i < len(active_config_list):
            flash_index, label, valid_types = active_config_list[i]
            print(f"{GREEN}Selected: {label} at index {flash_index}")
            break
        print(f"{RED}Invalid input. Please enter a valid number from the list.")

      if len(valid_types) == 1:
        config_type = valid_types[0]
      else:
        print(f"Valid types for this config: {valid_types}")
        config_type = None
        while True:
          ln = input(f"Enter the type to configure ({'/'.join(valid_types)}): ").strip().upper()
          if ln in valid_types:
            config_type = ln
            break
          print(f"{RED}Invalid config type. Choose one of: {', '.join(valid_types)}")

      if config_type == "CAN":
        print("Enter CAN config values:")
        can_id = int(input("can_id (decimal or 0xHEX): ").strip(), 0)
        scale_offs = int(input("scale_offs: ").strip())
        scale_mult = int(input("scale_mult: ").strip())
        msg_len_bytes = int(input("msg_len_bytes: ").strip())
        sig_type = int(input("sig_type: ").strip())
        shift_amt = int(input("shift_amt: ").strip())
        sig_len = int(input("sig_len: ").strip())
        endian_type = int(input("endian_type: ").strip())
        enabled = int(input("enabled: ").strip())
        is_signed = int(input("is_signed: ").strip())
        panda.flash_config_write_CAN(flash_index, can_id, scale_offs, scale_mult,
                                    msg_len_bytes, sig_type, shift_amt, sig_len,
                                    endian_type, enabled, is_signed)
        print(f"{GREEN}CAN config written.")

      elif config_type == "ADC":
        print("Enter ADC config values:")
        adc1 = int(input("adc1: ").strip())
        adc2 = int(input("adc2: ").strip())
        adc_tol = int(input("adc_tolerance: ").strip())
        adc_num = int(input("adc_num (0 or 1): ").strip())
        adc_en = int(input("adc_en (0 or 1): ").strip())
        panda.flash_config_write_ADC(flash_index, adc1, adc2, adc_tol, adc_num, adc_en)
        print(f"{GREEN}ADC config written.")

      elif config_type == "SYS":
        print("Enter SYS config values:")
        debug_lvl = int(input("debug_lvl: ".strip()))
        can_out_en = int(input("can_out_en: ".strip()))
        iwdg_en = int(input("iwdg_en (0 or 1): ".strip()))
        panda.flash_config_write_SYS(debug_lvl, can_out_en, iwdg_en)
        print(f"{GREEN}SYS config written.")
      
      elif config_type == "VSS":
        print("Enter VSS config values:")
        vss_ppd = int(input("VSS PPM or PPK: ".strip()))
        is_kph = int(input("Metric? 1=YES, 0=NO ".strip()))
        panda.flash_config_write_VSS(flash_index, vss_ppd, is_kph)
        print(f"{GREEN}VSS config written.")

      elif config_type == "RELAY":
        print("Enter Relay config values:")

        for idx, label in enumerate(relay_core_config_tags):
          print(f"{idx}: {label}")

        while True:
          ln = input("Enter the number of the label to apply: ").strip()
          if ln.isdigit():
            i = int(ln)
            if 0 <= i < len(relay_core_config_tags):
              selected_label = relay_core_config_tags[i]
              print(f"{GREEN}Selected: {selected_label}")
              break
          print(f"{RED}Invalid input. Please enter a valid number from the list.")
        
        gpio_en = int(input("gpio_en (0 or 1): ").strip())
        gpio_in = int(input("gpio_in: ").strip())
        can_addr = int(input("can_addr (hex, e.g. 0x123). A value of 0 disables CAN input: ").strip(), 0)
        sig_len = int(input("sig_len: ").strip())
        shift_amt = int(input("shift_amt: ").strip())
        can_cmp = int(input("can_cmp_val: ").strip(), 0)
        
        panda.flash_config_write_RELAY(flash_index, selected_label, gpio_en, gpio_in, can_addr, sig_len, shift_amt, can_cmp)
        print(f"{GREEN}RELAY config written.")

    except Exception as e:
      import traceback
      print(f"{RED}Error during operation: {e}")
      traceback.print_exc()