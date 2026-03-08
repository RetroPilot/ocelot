#!/usr/bin/env python3

import sys
import os
import time
import usb1
import json

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from firmware.python import Panda

def connect_chimera():
    context = usb1.USBContext()
    for device in context.getDeviceList(skip_on_error=True):
        if device.getVendorID() == 0xbbaa and device.getProductID() in [0xddcc, 0xddee]:
            try:
                handle = device.open()
                manufacturer = handle.getManufacturer()
                product = handle.getProduct()
                serial = handle.getSerialNumber()
                handle.close()
                
                if manufacturer != "commaai" and "Chimera" in product:
                    panda = Panda(serial=serial)
                    print(f"Connected to Chimera: {serial} ({product})")
                    return panda
            except Exception as e:
                print(f"Failed to connect: {e}")
    raise Exception("No Chimera found")

def get_chimera_values(panda):
    try:
        request = json.dumps({"method": "chimera.get_values"})
        panda._handle.controlWrite(0x40, 0xFC, 0, 0, request.encode('utf-8'))
        
        time.sleep(0.01)
        
        full_response = b''
        offset = 0
        chunk_size = 64
        
        while True:
            chunk = panda._handle.controlRead(0xC0, 0xFB, offset, 0, chunk_size)
            if not chunk or len(chunk) == 0:
                break
            full_response += bytes(chunk)
            if len(chunk) < chunk_size:
                break
            offset += len(chunk)
        
        if full_response:
            response_str = full_response.decode('utf-8').rstrip('\x00')
            response = json.loads(response_str)
            if 'result' in response:
                return response['result']
    except Exception as e:
        print(f"Error: {e}")
    return None

def main():
    print("Chimera JSON RPC Monitor")
    print("========================")
    
    try:
        panda = connect_chimera()
        print("Press Ctrl+C to stop\n")
        
        while True:
            values = get_chimera_values(panda)
            if values:
                print(f"ADC0: {values.get('adc0', 'N/A'):4d}  ADC1: {values.get('adc1', 'N/A'):4d}  "
                      f"VSS: {values.get('vss', 'N/A'):3d} km/h  RPM: {values.get('eng_rpm', 'N/A'):4d}  "
                      f"Steer: {values.get('steer_angle', 'N/A'):4d}°  "
                      f"IGN: {'ON' if values.get('ignition') else 'OFF'}  "
                      f"BRK: {'YES' if values.get('brake_pressed') else 'NO'}")
            else:
                print("No response")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopped")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            panda.close()
        except:
            pass

if __name__ == "__main__":
    main()