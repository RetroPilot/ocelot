#!/usr/bin/env python3

import usb1
import json

def find_relay_core():
    with usb1.USBContext() as context:
        for device in context.getDeviceIterator(skip_on_error=True):
            if device.getVendorID() == 0xbbaa and device.getProductID() == 0xddcc:
                try:
                    handle = device.open()
                    handle.claimInterface(0)
                    response = json_rpc_call(handle, "system.info")
                    if response and response.get("result", {}).get("device") == "relay_core":
                        return handle
                    handle.close()
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
        
        response_data = handle.controlRead(0xC0, 0xFB, 0, 0, 512)
        response_str = response_data.decode('utf-8').rstrip('\x00')
        
        if response_str:
            print(f"← {response_str}")
            return json.loads(response_str)
        return None
    except Exception as e:
        print(f"Error: {e}")
        return None

def test_flash_config():
    handle = find_relay_core()
    if not handle:
        print("Relay core not found!")
        return False
    
    print("Found relay core")
    
    try:
        # Format flash
        print("\n=== Format Flash ===")
        response = json_rpc_call(handle, "flash.format")
        if not response or response.get("error"):
            print("❌ Format failed")
            return False
        print("✅ Formatted")
        
        # Write config
        print("\n=== Write Config ===")
        config = {"index": 1, "cfg_type": 5, "gpio_en": 1, "gpio_in": 15}
        response = json_rpc_call(handle, "flash.write", config)
        if not response or response.get("error"):
            print("❌ Write failed")
            return False
        print("✅ Written")
        
        # Read back
        print("\n=== Read Config ===")
        response = json_rpc_call(handle, "flash.read", {"index": 1})
        if not response or response.get("error"):
            print("❌ Read failed")
            return False
        
        result = response.get("result", {})
        print(f"✅ Read: {result}")
        
        # Verify
        if (result.get("cfg_type") == 5 and result.get("gpio_en") == 1 and result.get("gpio_in") == 15):
            print("✅ Data matches!")
            return True
        else:
            print("❌ Data mismatch!")
            return False
    finally:
        handle.close()

if __name__ == "__main__":
    print("Testing Flash Config via JSON RPC")
    success = test_flash_config()
    print(f"\n{'🎉 Success!' if success else '💥 Failed!'}")