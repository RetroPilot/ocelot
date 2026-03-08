#!/usr/bin/env python3
import usb1
import json
import sys

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
        # Find device
        for device in context.getDeviceIterator(skip_on_error=True):
            if device.getVendorID() == 0xbbaa and device.getProductID() == 0xddcc:
                handle = device.open()
                break
        
        if not handle:
            print("Device not found. Make sure your RetroPilot device is connected.")
            return
        
        print("Found RetroPilot device")
        print("Ready to test JSON RPC")
        
        # Test JSON RPC methods organized by device type
        if len(sys.argv) > 1:
            device_type = sys.argv[1]
            command = sys.argv[2]
            
            # System commands
            if device_type == "system":
                if command == "info":
                    print("\n=== Testing system.info ===")
                    json_data = json.dumps({"method": "system.info"}).encode('utf-8')
                elif command == "methods":
                    print("\n=== Testing system.methods ===")
                    json_data = json.dumps({"method": "system.methods"}).encode('utf-8')
                elif command == "reset":
                    print("\n=== Testing system.reset (WARNING: This will reset the device!) ===")
                    confirm = input("Are you sure you want to reset the device? (y/N): ")
                    if confirm.lower() != 'y':
                        print("Reset cancelled")
                        return
                    json_data = json.dumps({"method": "system.reset"}).encode('utf-8')
                else:
                    print(f"Unknown system command: {command}")
                    print("Available system commands: info, methods, reset")
                    return
            
            # Actuator Core commands
            elif device_type == "actuator_core":
                if command == "status":
                    print("\n=== Testing actuator.get_status ===")
                    json_data = json.dumps({"method": "actuator.get_status"}).encode('utf-8')
                elif command == "motor":
                    if len(sys.argv) > 5:
                        motor_num = int(sys.argv[3])
                        pwm = int(sys.argv[4])
                        direction = int(sys.argv[5])
                        print(f"\n=== Testing actuator.motor motor_num={motor_num} pwm={pwm} dir={direction} ===")
                        json_data = json.dumps({"method": "actuator.motor", "params": {"m": motor_num, "p": pwm, "d": direction}}, separators=(',', ':')).encode('utf-8')
                    else:
                        print("Usage: actuator_core motor <motor_num> <pwm> <direction>")
                        return
                elif command == "relay":
                    if len(sys.argv) > 3:
                        value = int(sys.argv[3])
                        print(f"\n=== Testing actuator.relay value={value} ===")
                        json_data = json.dumps({"method": "actuator.relay", "params": {"value": value}}, separators=(',', ':')).encode('utf-8')
                    else:
                        print("Usage: actuator_core relay <value>")
                        return
                else:
                    print(f"Unknown actuator_core command: {command}")
                    print("Available actuator_core commands: status, motor <m> <p> <d>, relay <value>")
                    return
            
            # Relay Core commands
            elif device_type == "relay_core":
                if command == "set":
                    if len(sys.argv) > 4:
                        relay_num = int(sys.argv[3])
                        state = int(sys.argv[4])
                        print(f"\n=== Testing relay.set relay={relay_num} state={state} ===")
                        json_data = json.dumps({"method": "relay.set", "params": {"relay": relay_num, "state": state}}, separators=(',', ':')).encode('utf-8')
                    else:
                        print("\n=== Testing relay.set (polling) ===")
                        json_data = json.dumps({"method": "relay.set"}, separators=(',', ':')).encode('utf-8')
                elif command == "status":
                    print("\n=== Testing relay.set (polling) ===")
                    json_data = json.dumps({"method": "relay.set"}, separators=(',', ':')).encode('utf-8')
                else:
                    print(f"Unknown relay_core command: {command}")
                    print("Available relay_core commands: set [relay] [state], status")
                    return
            
            # Chimera commands
            elif device_type == "chimera":
                if command == "values":
                    print("\n=== Testing chimera.get_values ===")
                    json_data = json.dumps({"method": "chimera.get_values"}).encode('utf-8')
                else:
                    print(f"Unknown chimera command: {command}")
                    print("Available chimera commands: values")
                    return
            
            # Interceptor Core commands
            elif device_type == "interceptor_core":
                if command == "values":
                    print("\n=== Testing interceptor.get_values ===")
                    json_data = json.dumps({"method": "interceptor.get_values"}).encode('utf-8')
                else:
                    print(f"Unknown interceptor_core command: {command}")
                    print("Available interceptor_core commands: values")
                    return
            
            # Test command (runs full test sequence)
            elif device_type == "test" and command == "all":
                print("\n=== Running full test sequence ===")
                test_commands = [
                    ({"method": "system.info"}, "Device Info"),
                    ({"method": "system.methods"}, "Available Methods"),
                    ({"method": "actuator.get_status"}, "Actuator Status"),
                    ({"method": "actuator.motor", "params": {"m": 1, "p": 25, "d": 1}}, "Motor 1: 25% Forward"),
                    ({"method": "actuator.motor", "params": {"m": 1, "p": 0, "d": 1}}, "Motor 1: Stop"),
                    ({"method": "actuator.motor", "params": {"m": 2, "p": 50, "d": 0}}, "Motor 2: 50% Reverse"),
                    ({"method": "actuator.motor", "params": {"m": 2, "p": 0, "d": 0}}, "Motor 2: Stop"),
                    ({"method": "actuator.relay", "params": {"value": 1}}, "Actuator Relay ON"),
                    ({"method": "actuator.relay", "params": {"value": 0}}, "Actuator Relay OFF"),
                    ({"method": "relay.set"}, "Poll Relay State (if relay core)"),
                    ({"method": "relay.set", "params": {"relay": 1, "state": 1}}, "Set Relay 1 ON (if relay core)"),
                    ({"method": "relay.set", "params": {"relay": 1, "state": 0}}, "Set Relay 1 OFF (if relay core)"),
                    ({"method": "chimera.get_values"}, "Chimera Values (if chimera)"),
                    ({"method": "interceptor.get_values"}, "Interceptor Values (if interceptor)"),
                    ({"method": "flash.read", "params": {"index": 1}}, "Flash Read (should fail)"),
                    ({"method": "unknown.method"}, "Unknown Method (should fail)")
                ]
                
                for cmd, desc in test_commands:
                    print(f"\n--- {desc} ---")
                    try:
                        json_data = json.dumps(cmd).encode('utf-8')
                        result = handle.controlWrite(0x40, 0xFC, 0, 0, json_data)
                        response = read_chunked_json(handle)
                        if response and 'result' in response:
                            print(f"Result: {json.dumps(response['result'], indent=2)}")
                    except Exception as e:
                        print(f"Error: {e}")
                return
            
            else:
                print(f"Unknown device type: {device_type}")
                print("Available device types: system, actuator_core, relay_core, chimera, interceptor_core, test")
                return
            
            try:
                # Send command - now returns response directly
                response_data = handle.controlWrite(0x40, 0xFC, 0, 0, json_data)
                print(f"Command sent (bytes: {response_data})")
                print(f"Sent JSON: {json_data.decode('utf-8')}")
                
                # Wait for device to process command
                import time
                time.sleep(0.05)  # Longer delay
                response = read_chunked_json(handle)
                
                if response:
                    print(f"\nResponse: {json.dumps(response, indent=2)}")
                    
                    # Pretty print specific results
                    if 'result' in response and response['result']:
                        result_data = response['result']
                        if command == "info" and isinstance(result_data, dict):
                            print(f"\nDevice: {result_data.get('device', 'unknown')}")
                            print(f"Version: {result_data.get('version', 'unknown')}")
                            print(f"Features: {', '.join(result_data.get('features', []))}")
                        elif command == "methods" and isinstance(result_data, list):
                            print(f"\nAvailable methods ({len(result_data)}):")
                            for m in result_data:
                                print(f"  - {m}")
                        elif command == "status" and isinstance(result_data, dict):
                            if device_type == "actuator_core":
                                print(f"\nActuator Status:")
                                print(f"  Motor 1: PWM={result_data.get('motor1_pwm', 'N/A')}%, Dir={result_data.get('motor1_dir', 'N/A')}, Enabled={result_data.get('motor1_enable', 'N/A')}")
                                print(f"  Motor 2: PWM={result_data.get('motor2_pwm', 'N/A')}%, Dir={result_data.get('motor2_dir', 'N/A')}, Enabled={result_data.get('motor2_enable', 'N/A')}")
                                print(f"  Relay: {'ON' if result_data.get('relay_state') else 'OFF'}")
                                print(f"  ADC0: {result_data.get('adc0', 'N/A')}")
                                print(f"  ADC1: {result_data.get('adc1', 'N/A')}")
                                print(f"  State: {result_data.get('state', 'N/A')}")
                                print(f"  Mode: {result_data.get('mode', 'N/A')}")
                                print(f"  USB Control: {'Active' if result_data.get('usb_ctrl') else 'Inactive'}")
                            else:
                                print(f"\nStatus: {json.dumps(result_data, indent=2)}")
                        elif command == "values" and isinstance(result_data, dict):
                            print(f"\nSensor Values:")
                            # Handle both Chimera and Interceptor formats
                            if 'steer_angle' in result_data:  # Chimera format
                                print(f"  ADC0: {result_data.get('adc0', 'N/A')}")
                                print(f"  ADC1: {result_data.get('adc1', 'N/A')}")
                                print(f"  Vehicle Speed: {result_data.get('vehicle_speed', 'N/A')} km/h")
                                print(f"  Engine RPM: {result_data.get('engine_rpm', 'N/A')}")
                                print(f"  Steering Angle: {result_data.get('steer_angle', 'N/A')}°")
                                print(f"  Steering Rate: {result_data.get('steer_rate', 'N/A')}°/s")
                                print(f"  Ignition: {'ON' if result_data.get('ignition') else 'OFF'}")
                                print(f"  Brake Pressed: {'YES' if result_data.get('brake_pressed') else 'NO'}")
                                print(f"  Buttons: {result_data.get('buttons', 'N/A')}")
                            else:  # Interceptor format
                                print(f"  ADC0: {result_data.get('adc0', 'N/A')}")
                                print(f"  ADC1: {result_data.get('adc1', 'N/A')}")
                                print(f"  DAC0: {result_data.get('dac0', 'N/A')}")
                                print(f"  DAC1: {result_data.get('dac1', 'N/A')}")
                                print(f"  Mode: {result_data.get('mode', 'N/A')}")
                                print(f"  State: {result_data.get('state', 'N/A')}")
                                print(f"  Enabled: {'YES' if result_data.get('enabled') else 'NO'}")
                                print(f"  USB Control: {'Active' if result_data.get('usb_ctrl') else 'Inactive'}")
                else:
                    print("No response received")
                
            except Exception as e:
                print(f"Error: {e}")
        else:
            print("\nUsage: python test_usb_endpoint.py <device_type> <command> [args]")
            print("\nDevice Types and Commands:")
            print("\n  system:")
            print("    info                                    # Get device info")
            print("    methods                                 # List all methods")
            print("    reset                                   # Reset device (with confirmation)")
            print("\n  actuator_core:")
            print("    status                                  # Get actuator status")
            print("    motor <motor_num> <pwm> <direction>     # Control motor")
            print("    relay <value>                           # Control actuator relay")
            print("\n  relay_core:")
            print("    status                                  # Poll current relay state")
            print("    set [relay_num] [state]                 # Set relay state")
            print("\n  chimera:")
            print("    values                                  # Get sensor values")
            print("\n  interceptor_core:")
            print("    values                                  # Get sensor values")
            print("\n  test:")
            print("    all                                     # Run full test sequence")
            print("\nExamples:")
            print("  python test_usb_endpoint.py system info")
            print("  python test_usb_endpoint.py actuator_core status")
            print("  python test_usb_endpoint.py actuator_core motor 1 50 1")
            print("  python test_usb_endpoint.py actuator_core relay 1")
            print("  python test_usb_endpoint.py relay_core set 1 1")
            print("  python test_usb_endpoint.py relay_core status")
            print("  python test_usb_endpoint.py chimera values")
            print("  python test_usb_endpoint.py interceptor_core values")
            print("  python test_usb_endpoint.py test all")
            print("\nParameter Details:")
            print("  Motor: motor_num=1|2, pwm=0-100, direction=0(reverse)|1(forward)")
            print("  Relay: relay_num=1-8, state=0(off)|1(on), value=0(off)|1(on)")
    
    finally:
        if handle:
            handle.close()
        context.close()

if __name__ == "__main__":
    main()