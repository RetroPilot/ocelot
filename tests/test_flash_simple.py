#!/usr/bin/env python3

import usb1
import json
import time

def find_relay_core():
    context = usb1.USBContext()
    for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == 0xbbaa and device.getProductID() == 0xddcc:
            try:
                handle = device.open()
                handle.claimInterface(0)
                return handle
            except:
                continue
    return None

def json_rpc_call(handle, method, params=None):
    try:
        request = {"method": method}
        if params:
            request["params"] = params
        
        request_str = json.dumps(request)
        print(f"→ {request_str}")
        
        handle.controlWrite(0x40, 0xFC, 0, 0, request_str.encode('utf-8'))
        time.sleep(0.1)  # Give device time to process
        
        response_data = handle.controlRead(0xC0, 0xFB, 0, 0, 512)
        response_str = response_data.decode('utf-8').rstrip('\x00')
        
        if response_str:
            print(f"← {response_str}")
            return json.loads(response_str)
        return None
    except Exception as e:
        print(f"Error: {e}")
        return None

if __name__ == "__main__":
    handle = find_relay_core()
    if not handle:
        print("Device not found!")
        exit(1)
    
    try:
        # Test system.info first
        print("=== System Info ===")
        response = json_rpc_call(handle, "system.info")
        
        # Test flash.format
        print("\n=== Format Flash ===")
        response = json_rpc_call(handle, "flash.format")
        
        # Test flash.write
        print("\n=== Write Flash Config ===")
        config = {"index": 1, "cfg_type": 5, "gpio_en": 1, "gpio_in": 15}
        response = json_rpc_call(handle, "flash.write", config)
        
        # Test flash.read to verify
        print("\n=== Read Flash Slot 1 ===")
        response = json_rpc_call(handle, "flash.read", {"index": 1})
        
    finally:
        handle.close()