#!/usr/bin/env python3
import usb1
import json
import sys
import time

def read_chunked_json(handle):
    full_response = b''
    offset = 0
    chunk_size = 64
    
    while True:
        chunk = handle.controlRead(0xC0, 0xFB, offset, 0, chunk_size)
        if not chunk or len(chunk) == 0:
            break
        full_response += bytes(chunk)
        if len(chunk) < chunk_size:
            break
        offset += len(chunk)
    
    if full_response:
        return json.loads(full_response.decode('utf-8').rstrip('\x00'))
    return None

def main():
    context = usb1.USBContext()
    handle = None
    
    try:
        # Find interceptor device
        for device in context.getDeviceIterator(skip_on_error=True):
            if device.getVendorID() == 0xbbaa and device.getProductID() == 0xddcc:
                handle = device.open()
                break
        
        if not handle:
            print("Interceptor Core not found")
            return
        
        print("Found Interceptor Core")
        
        # Test interceptor methods
        if len(sys.argv) > 1:
            method = sys.argv[1]
            
            if method == "info":
                json_data = json.dumps({"method": "system.info"}).encode('utf-8')
            elif method == "methods":
                json_data = json.dumps({"method": "system.methods"}).encode('utf-8')
            elif method == "values":
                json_data = json.dumps({"method": "interceptor.get_values"}).encode('utf-8')
            elif method == "dac" and len(sys.argv) > 3:
                dac0 = int(sys.argv[2])
                dac1 = int(sys.argv[3])
                json_data = json.dumps({"method": "test_dac", "params": {"dac0": dac0, "dac1": dac1}}).encode('utf-8')
            elif method == "stop":
                json_data = json.dumps({"method": "interceptor.emergency_stop"}).encode('utf-8')
            else:
                print("Usage: python test_interceptor_json.py <method> [args]")
                print("Methods: info, methods, values, dac <dac0> <dac1>, stop")
                return
            
            # Send command
            handle.controlWrite(0x40, 0xFC, 0, 0, json_data)
            time.sleep(0.05)
            response = read_chunked_json(handle)
            
            if response:
                print(f"Response: {json.dumps(response, indent=2)}")
            else:
                print("No response received")
        else:
            print("Usage: python test_interceptor_json.py <method> [args]")
            print("Methods: info, methods, values, dac <dac0> <dac1>, stop")
            print("Examples:")
            print("  python test_interceptor_json.py info")
            print("  python test_interceptor_json.py values")
            print("  python test_interceptor_json.py dac 2000 2500")
            print("  python test_interceptor_json.py stop")
    
    finally:
        if handle:
            handle.close()
        context.close()

if __name__ == "__main__":
    main()