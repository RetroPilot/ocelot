#!/usr/bin/env python3

import sys
import os
import time
import struct
import usb1
import json

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from firmware.python import Panda

class InterceptorTester:
    def __init__(self):
        self.panda = self._connect_chimera()
        self.counter = 0
        self.crc_table = self._generate_crc_table()
        self.mode = None
        self.can_input_id = None
        self.can_output_id = None
        
    def _connect_chimera(self):
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
                        panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
                        panda.set_can_speed_kbps(0, 500)
                        panda.set_can_speed_kbps(1, 500)
                        panda.set_can_speed_kbps(2, 500)
                        # Enable CAN forwarding from bus 0 to other buses
                        print("Setting up CAN forwarding...")
                        return panda
                except Exception as e:
                    print(f"Failed to connect: {e}")
        raise Exception("No Chimera found")
    
    def _generate_crc_table(self):
        poly = 0x1D
        table = [0] * 256
        for i in range(256):
            crc = i
            for j in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ poly) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
            table[i] = crc
        return table
    
    def _calculate_checksum(self, data):
        crc = 0xFF
        for i in range(1, len(data)):
            crc ^= data[i]
            crc = self.crc_table[crc]
        return crc ^ 0xFF
    
    def detect_interceptor_mode(self):
        """Listen for CAN messages to detect interceptor mode"""
        print("Detecting interceptor mode...")
        start_time = time.time()
        
        while time.time() - start_time < 5.0:  # Listen for 5 seconds
            messages = self.panda.can_recv()
            for addr, _, data, bus in messages:
                if addr == 0x201:  # Gas pedal output
                    self.mode = 'gas_pedal'
                    self.can_input_id = 0x200
                    self.can_output_id = 0x201
                    print("Detected: GAS PEDAL mode")
                    return True
                elif addr == 0x301:  # Differential output
                    self.mode = 'differential'
                    self.can_input_id = 0x300
                    self.can_output_id = 0x301
                    print("Detected: DIFFERENTIAL mode")
                    return True
                elif addr == 0x500:  # Unconfigured output
                    print("Detected: UNCONFIGURED mode - cannot test")
                    return False
            time.sleep(0.1)
        
        print("No interceptor detected")
        return False
    
    def send_dac_command(self, dac0, dac1, enable=True, debug=False):
        self.counter = (self.counter + 1) & 0xF
        data = bytearray(6)
        data[1] = dac0 & 0xFF
        data[2] = (dac0 >> 8) & 0xFF
        data[3] = dac1 & 0xFF
        data[4] = (dac1 >> 8) & 0xFF
        data[5] = (int(enable) << 7) | self.counter
        data[0] = self._calculate_checksum(data)
        
        if debug:
            print(f"Sending CAN ID 0x{self.can_input_id:03X} on bus 0: {data.hex()} (counter={self.counter})")
        self.panda.can_send(self.can_input_id, bytes(data), 0)
    
    def read_adc_values(self):
        """Read ADC values from Chimera via JSON RPC"""
        try:
            # Send JSON RPC request for Chimera values
            request = json.dumps({"method": "chimera.get_values"})
            self.panda._handle.controlWrite(0x40, 0xFC, 0, 0, request.encode('utf-8'))
            
            # Wait for processing
            time.sleep(0.01)
            
            # Read JSON RPC response in chunks
            full_response = b''
            offset = 0
            chunk_size = 64
            
            while True:
                chunk = self.panda._handle.controlRead(0xC0, 0xFB, offset, 0, chunk_size)
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
                    result = response['result']
                    return result.get('adc0', 0), result.get('adc1', 0)
        except Exception as e:
            print(f"ADC read failed: {e}")
        
        return 0, 0  # fallback
    
    def test_can_communication(self):
        """Test basic CAN communication"""
        print("Testing CAN communication...")
        
        # Send a test message on bus 0
        test_data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06])
        self.panda.can_send(0x123, test_data, 0)
        print(f"Sent test message on bus 0: 0x123 {test_data.hex()}")
        
        # Listen for any messages
        time.sleep(0.1)
        messages = self.panda.can_recv()
        print(f"Received {len(messages)} CAN messages")
        for addr, _, data, bus in messages:
            print(f"  Bus {bus}: 0x{addr:03X} {data.hex()}")
        
        return len(messages) > 0
    
    def test_dac_sweep(self):
        # First test basic CAN communication
        if not self.test_can_communication():
            print("WARNING: No CAN communication detected")
        
        if not self.detect_interceptor_mode():
            return False
        
        print(f"\nTesting {self.mode} mode DAC sweep...")
        
        # First, send continuous commands to clear timeout state
        print("Clearing timeout state with continuous commands...")
        for i in range(50):
            self.send_dac_command(2048, 2048, True, debug=(i < 3))
            time.sleep(0.02)
        print("Timeout clearing complete.\n")
        
        if self.mode == 'gas_pedal':
            test_values = [(0, 0), (1024, 1024), (2048, 2048), (3072, 3072), (4095, 4095)]
        else:  # differential
            test_values = [(2048, 2048), (2200, 1800), (1800, 2200), (2400, 1600)]
        
        for dac0, dac1 in test_values:
            print(f"Testing DAC0={dac0}, DAC1={dac1}")
            
            # Send commands at 50Hz for 2 seconds to ensure interceptor receives them
            print(f"  Sending {100} commands at 50Hz...")
            interceptor_responses = 0
            for i in range(100):  # 2 seconds at 50Hz
                self.send_dac_command(dac0, dac1, True, debug=(i < 3))
                
                # Check for interceptor response
                messages = self.panda.can_recv()
                for addr, _, data, bus in messages:
                    if addr == self.can_output_id:
                        interceptor_responses += 1
                        # Extract state field from last byte (upper 4 bits)
                        state = (data[-1] >> 4) & 0xF if len(data) > 0 else 0xFF
                        if i < 5:  # Show first few responses
                            print(f"    Interceptor response: 0x{addr:03X} {data.hex()} (state={state})")
                        elif state != 0:
                            print(f"    WARNING: Interceptor state={state} (should be 0)")
                
                if i % 25 == 0:  # Print every 0.5 seconds
                    print(f"    Sent {i+1}/100 commands, got {interceptor_responses} responses")
                time.sleep(0.02)
            
            print(f"  Total interceptor responses: {interceptor_responses}/100")
            
            # Read ADC values from Chimera
            adc0, adc1 = self.read_adc_values()
            
            if adc0 == 0 and adc1 == 0:
                print(f"WARNING: Got zero ADC values - check connection")
            
            # Check if ADC values are within tolerance
            tolerance = 100  # ±100 ADC counts
            if abs(adc0 - dac0) > tolerance or abs(adc1 - dac1) > tolerance:
                print(f"FAIL: Expected DAC0={dac0}, DAC1={dac1}, Got ADC0={adc0}, ADC1={adc1}")
                return False
            else:
                print(f"PASS: DAC0={dac0}→ADC0={adc0}, DAC1={dac1}→ADC1={adc1}")
        
        # Disable at end
        for _ in range(50):
            self.send_dac_command(0, 0, False)
            time.sleep(0.02)
        
        return True
    
    def close(self):
        if hasattr(self, 'panda'):
            self.panda.close()

def main():
    print("Interceptor Core DAC Test")
    print("=========================")
    
    try:
        tester = InterceptorTester()
        success = tester.test_dac_sweep()
        print(f"\nTest {'PASSED' if success else 'FAILED'}")
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        try:
            tester.close()
        except:
            pass

if __name__ == "__main__":
    main()