#!/usr/bin/env python3

import os
import sys
import usb1
from firmware.python import Panda

RED = '\033[31m'
GREEN = '\033[32m'

CHIMERA_PANDA_VID = 0xbbaa
CHIMERA_PANDA_PIDS = [0xddcc, 0xddee]  # Panda + Chimera variants

SUPPORTED_PRODUCTS = ["Chimera", "Actuator Core", "Relay Core", "Interceptor Core"]

# Unified configuration list (index, label, supported types)
chimera_config_list = [
  (0,  "System Config",           ["SYS"]),
  (1,  "Steer Angle Major",       ["CAN"]),
  (2,  "Steer Angle Minor",       ["CAN"]),
  (3,  "Steer Angle Rate",        ["CAN"]),
  (4,  "Vehicle Speed",           ["CAN", "HALL"]),
  (5,  "Ignition",                ["CAN"]),
  (6,  "Engine RPM",              ["CAN", "HALL"]),
  (7,  "Brake Pressed",           ["CAN"]),
  (11, "Cruise Button Cancel",    ["CAN", "ADC"]),
  (12, "Cruise Button Set/Down",  ["CAN", "ADC"]),
  (13, "Cruise Button Res/Up",    ["CAN", "ADC"]),
  (14, "Cruise Button On/Off",    ["CAN", "ADC"]),
]

actuator_core_config_list = [
  (0,  "System Config",           ["SYS"]),
  (1,  "TPS Config",              ["ADC"]),
  (2,  "Motor Config",            ["MOTOR"]),
  (3,  "Clutch Config",           ["MOTOR"]),
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

interceptor_core_config_list = [
  (0, "System Config",            ["SYS"]),
  (1, "ADC Channel 0 Validation", ["ADC"]),
  (2, "ADC Channel 1 Validation", ["ADC"]),
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
      mode_names = {0: "Unconfigured", 1: "Differential", 2: "Gas Pedal"}
      mode_name = mode_names.get(e.get('mode', 0), "Unknown")
      print(f"[{idx}] SYSTEM: debug_lvl={e['debug_lvl']}, can_out={e['can_out_en']}, iwdg_en={e['iwdg_en']}, mode={e.get('mode', 0)} ({mode_name})")
    elif cfg_type == "HALL":
      print("HALL: ", e)
    elif cfg_type == "RELAY":
      label = get_relay_label_from_type(e['label'])
      print(f"[{idx}] RELAY: {label}, can_cmp={e['can_cmp_val']}, gpio_en={e['gpio_en']}, gpio_in={e['gpio_in']}, can_addr=0x{e['can_addr']:X}, sig_len={e['sig_len']}, shift={e['shift_amt']}")
    elif cfg_type == "MOTOR":
      type_names = {1: "Motor", 2: "Clutch"}
      polarity_names = {1: "Normal", 2: "Inverted"}
      type_name = type_names.get(e.get('type', 0), "Unknown")
      polarity_name = polarity_names.get(e.get('polarity', 0), "Unknown")
      print(f"[{idx}] MOTOR: {type_name}, Bridge={e.get('bridge_channel', 0)}, Polarity={polarity_name}")
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
        print(f"{GREEN}Relay Core device detected. Using Relay Core config.")
    elif "INTERCEPTOR CORE" in selected_product_name:
        active_config_list = interceptor_core_config_list
        print(f"{GREEN}Interceptor Core device detected. Using Interceptor Core config.")
    else:
        print(f"{RED}Unknown device type. Cannot determine configuration options.")
        sys.exit(1)


    device_serial = selected.get("serial")
    print(f"Attempting to connect to device with serial: {device_serial}")
    try:
      panda = Panda(serial=device_serial)
      print(f"{GREEN}Connected successfully.")
      
      print(f"{GREEN}What would you like to do?")
      print("0: Format flash (erase all configurations)")
      print("1: Configure device")
      
      while True:
        action = input("Enter your choice (0 or 1): ").strip()
        if action in ["0", "1"]:
          break
        print(f"{RED}Invalid choice. Please enter 0 or 1.")
      
      if action == "0":
        print(f"{GREEN}Formatting flash...")
        panda.flash_wipe_config()
        print("FLASH FORMATTED")
        panda.reset()
        continue
      
      # action == "1" - configure device
      print(f"{GREEN}Reading existing flash config:")
      try:
        flash_entries = panda.flash_config_read()
        print_config_entries(flash_entries)
      except Exception as e:
        print(f"{RED}Failed to read flash config: {e}")
        if str(e) == "Invalid config magic":
          print(f"{RED}Flash is not formatted. Please format first (option 0).")
          continue

      print(f"{GREEN}\nAvailable configuration options:")

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
        # Determine current mode for context-aware prompts
        current_mode = 0  # Default to unconfigured
        try:
          flash_entries = panda.flash_config_read()
          for entry in flash_entries:
            if entry.get('index') == 0 and entry.get('cfg_type') == 'SYSTEM':
              current_mode = entry.get('mode', 0)
              break
        except:
          pass
        
        mode_names = {0: "Unconfigured", 1: "Differential", 2: "Gas Pedal"}
        print(f"Current device mode: {mode_names.get(current_mode, 'Unknown')}")
        
        if current_mode == 1:  # Differential mode
          print("\nDifferential Mode ADC Validation (Center Point + Tolerance):")
          adc1_input = input("Center value (expected sensor position): ").strip()
          adc1 = int(adc1_input) if adc1_input else 2048
          adc2 = 0  # Unused in differential mode
          adc_tol_input = input("Tolerance (+/- deviation allowed): ").strip()
          adc_tol = int(adc_tol_input) if adc_tol_input else 100
        elif current_mode == 2:  # Gas pedal mode
          print("\nGas Pedal Mode ADC Validation (Range Limits):")
          adc2_input = input("Minimum valid value (e.g., 200 for 0.2V): ").strip()
          adc2 = int(adc2_input) if adc2_input else 200
          adc1_input = input("Maximum valid value (e.g., 4000 for 4.0V): ").strip()
          adc1 = int(adc1_input) if adc1_input else 4000
          adc_tol = int(input("Tolerance/hysteresis (optional): ").strip() or "0")
        else:  # Unconfigured mode
          print("\nGeneric ADC Configuration:")
          adc1_input = input("adc1: ").strip()
          adc1 = int(adc1_input) if adc1_input else 0
          adc2_input = input("adc2: ").strip()
          adc2 = int(adc2_input) if adc2_input else 0
          adc_tol_input = input("adc_tolerance: ").strip()
          adc_tol = int(adc_tol_input) if adc_tol_input else 0
        
        adc_num = flash_index - 1   # Channel 0 at index 1, Channel 1 at index 2
        adc_en = int(input("Enable validation (0=disabled, 1=enabled): ").strip())
        
        panda.flash_config_write_ADC(flash_index, adc1, adc2, adc_tol, adc_num, adc_en)
        print(f"{GREEN}ADC config written for channel {adc_num}.")
        
        if current_mode == 1:
          print(f"Configured: Center={adc1}, Tolerance=±{adc_tol}")
        elif current_mode == 2:
          print(f"Configured: Range={adc2}-{adc1}, Tolerance={adc_tol}")
        else:
          print(f"Configured: adc1={adc1}, adc2={adc2}, tolerance={adc_tol}")

      elif config_type == "SYS":
        print("Enter SYS config values:")
        debug_lvl = int(input("debug_lvl: ".strip()))
        can_out_en = int(input("can_out_en: ".strip()))
        iwdg_en = int(input("iwdg_en (0 or 1): ".strip()))
        
        # Mode selection for devices that support it
        mode = 0
        override_threshold = 0
        if "INTERCEPTOR CORE" in selected_product_name:
          print("\nMode selection:")
          print("0: Unconfigured (safe default)")
          print("1: Differential (torque interceptor)")
          print("2: Gas Pedal (pedal-style)")
          while True:
            mode_input = input("Enter mode (0-2): ").strip()
            if mode_input.isdigit() and 0 <= int(mode_input) <= 2:
              mode = int(mode_input)
              break
            print(f"{RED}Invalid mode. Please enter 0, 1, or 2.")
          
          # Override threshold for differential mode
          if mode == 1:  # Differential mode
            print("\nDifferential Override Threshold:")
            print("This sets when the system detects large steering input differences.")
            print("Default: 336 (0x150). Range: 1-65535")
            while True:
              threshold_input = input("Enter override threshold (or press Enter for default): ").strip()
              if threshold_input == "":
                override_threshold = 336  # Default 0x150
                break
              elif threshold_input.isdigit() and 1 <= int(threshold_input) <= 65535:
                override_threshold = int(threshold_input)
                break
              print(f"{RED}Invalid threshold. Please enter 1-65535 or press Enter for default.")
        elif "ACTUATOR CORE" in selected_product_name:
          print("\nMode selection:")
          print("0: Unconfigured (safe default)")
          print("1: Differential (torque interceptor)")
          print("2: Gas Pedal (pedal-style)")
          while True:
            mode_input = input("Enter mode (0-2): ").strip()
            if mode_input.isdigit() and 0 <= int(mode_input) <= 2:
              mode = int(mode_input)
              break
            print(f"{RED}Invalid mode. Please enter 0, 1, or 2.")
          
        panda.flash_config_write_SYS(debug_lvl, can_out_en, iwdg_en, mode, override_threshold)
        print(f"{GREEN}SYS config written.")
        if "INTERCEPTOR CORE" in selected_product_name and mode == 1:
          print(f"Override threshold set to: {override_threshold} (0x{override_threshold:X})")
      
      elif config_type == "HALL":
        print("Enter HALL config values:")
        vss_ppd = int(input("VSS PPM or PPK: ".strip()))
        is_kph = int(input("Metric? 1=YES, 0=NO ".strip()))
        rel_cnt = int(input("Reluctor tooth count: ".strip()))
        skipped_tooth = int(input("Skipped tooth count: ".strip()))
        panda.flash_config_write_HALL(flash_index, vss_ppd, is_kph, rel_cnt, skipped_tooth)
        print(f"{GREEN}HALL config written.")

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
        
        gpio_en = int(input("gpio_en (0 or 1) [0]: ").strip() or "0")
        gpio_in = int(input("gpio_bitmask [0]: ").strip() or "0")
        can_addr = int(input("can_addr (hex, e.g. 0x123). A value of 0 disables CAN input [0]: ").strip() or "0")
        sig_len = int(input("sig_len [0]: ").strip() or "0")
        shift_amt = int(input("shift_amt [0]: ").strip() or "0")
        can_cmp = int(input("can_cmp_val [0]: ").strip() or "0")
        
        panda.flash_config_write_RELAY(flash_index, selected_label, gpio_en, gpio_in, can_addr, sig_len, shift_amt, can_cmp)
        print(f"{GREEN}RELAY config written.")

      elif config_type == "MOTOR":
        print("Enter Motor config values:")
        
        # Determine if this is motor or clutch based on flash index
        if flash_index == 2:
          default_type = 1  # Motor
          print("Configuring throttle motor...")
        elif flash_index == 3:
          default_type = 2  # Clutch
          print("Configuring clutch...")
        else:
          print("Motor type selection:")
          print("1: Motor (throttle actuator)")
          print("2: Clutch (engagement mechanism)")
          while True:
            type_input = input("Enter type (1 or 2): ").strip()
            if type_input in ["1", "2"]:
              default_type = int(type_input)
              break
            print(f"{RED}Invalid type. Please enter 1 or 2.")
        
        print("\nBridge channel selection:")
        print("1: Bridge 1 (PWM on PA2)")
        print("2: Bridge 2 (PWM on PA3)")
        while True:
          bridge_input = input("Enter bridge channel (1 or 2): ").strip()
          if bridge_input in ["1", "2"]:
            bridge_channel = int(bridge_input)
            break
          print(f"{RED}Invalid bridge channel. Please enter 1 or 2.")
        
        print("\nPolarity selection:")
        print("1: Normal (standard direction)")
        print("2: Inverted (reversed direction)")
        while True:
          polarity_input = input("Enter polarity (1 or 2): ").strip()
          if polarity_input in ["1", "2"]:
            polarity = int(polarity_input)
            break
          print(f"{RED}Invalid polarity. Please enter 1 or 2.")
        
        panda.flash_config_write_MOTOR(flash_index, bridge_channel, default_type, polarity)
        type_names = {1: "Motor", 2: "Clutch"}
        polarity_names = {1: "Normal", 2: "Inverted"}
        print(f"{GREEN}MOTOR config written: {type_names[default_type]}, Bridge {bridge_channel}, {polarity_names[polarity]}")

    except Exception as e:
      import traceback
      print(f"{RED}Error during operation: {e}")
      traceback.print_exc()