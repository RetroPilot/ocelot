#!/usr/bin/env python3
import usb1
import json
import sys
import time
import threading
import tty
import termios

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

def send_motor_command(handle, motor_num, pwm, direction):
    json_data = json.dumps({
        "method": "actuator.motor",
        "params": {"m": motor_num, "p": pwm, "d": direction}
    }, separators=(',', ':')).encode('utf-8')
    handle.controlWrite(0x40, 0xFC, 0, 0, json_data)

def send_relay_command(handle, value):
    json_data = json.dumps({
        "method": "actuator.relay",
        "params": {"value": value}
    }, separators=(',', ':')).encode('utf-8')
    handle.controlWrite(0x40, 0xFC, 0, 0, json_data)

def keepalive_thread(handle, motor_states, stop_event):
    while not stop_event.is_set():
        for motor_num, state in motor_states.items():
            send_motor_command(handle, motor_num, state['pwm'], state['dir'])
        time.sleep(0.3)

def print_status(motor_states, relay_state):
    sys.stdout.write(f"\rM1: {motor_states[1]['pwm']:3d}% {'FWD' if motor_states[1]['dir'] else 'REV'}  |  M2: {motor_states[2]['pwm']:3d}% {'FWD' if motor_states[2]['dir'] else 'REV'}  |  Relay: {'ON' if relay_state else 'OFF'}  ")
    sys.stdout.flush()

def main():
    context = usb1.USBContext()
    handle = None
    
    try:
        for device in context.getDeviceIterator(skip_on_error=True):
            if device.getVendorID() == 0xbbaa and device.getProductID() == 0xddcc:
                handle = device.open()
                break
        
        if not handle:
            print("Device not found. Make sure Actuator Core is connected.")
            return
        
        print("=== Actuator Core Motor Control ===")
        print("Controls:")
        print("  W/S       - Motor 1 PWM +/-")
        print("  Up/Down   - Motor 2 PWM +/-")
        print("  1         - Toggle Motor 1 direction")
        print("  2         - Toggle Motor 2 direction")
        print("  R         - Toggle Relay")
        print("  Space     - Stop all motors")
        print("  Q         - Quit")
        print()
        
        motor_states = {1: {'pwm': 0, 'dir': 1}, 2: {'pwm': 0, 'dir': 1}}
        relay_state = 0
        stop_event = threading.Event()
        keepalive = threading.Thread(target=keepalive_thread, args=(handle, motor_states, stop_event), daemon=True)
        keepalive.start()
        
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            print_status(motor_states, relay_state)
            
            while True:
                ch = sys.stdin.read(1)
                
                if ch == 'q' or ch == 'Q':
                    break
                elif ch == 'w' or ch == 'W':
                    motor_states[1]['pwm'] = min(100, motor_states[1]['pwm'] + 1)
                elif ch == 's' or ch == 'S':
                    motor_states[1]['pwm'] = max(0, motor_states[1]['pwm'] - 1)
                elif ch == '\x1b':
                    next1 = sys.stdin.read(1)
                    if next1 == '[':
                        next2 = sys.stdin.read(1)
                        if next2 == 'A':  # Up arrow
                            motor_states[2]['pwm'] = min(100, motor_states[2]['pwm'] + 1)
                        elif next2 == 'B':  # Down arrow
                            motor_states[2]['pwm'] = max(0, motor_states[2]['pwm'] - 1)
                elif ch == '1':
                    motor_states[1]['dir'] = 1 - motor_states[1]['dir']
                elif ch == '2':
                    motor_states[2]['dir'] = 1 - motor_states[2]['dir']
                elif ch == 'r' or ch == 'R':
                    relay_state = 1 - relay_state
                    send_relay_command(handle, relay_state)
                elif ch == ' ':
                    motor_states[1]['pwm'] = 0
                    motor_states[2]['pwm'] = 0
                
                print_status(motor_states, relay_state)
        
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        
        stop_event.set()
        motor_states[1] = {'pwm': 0, 'dir': 1}
        motor_states[2] = {'pwm': 0, 'dir': 1}
        send_motor_command(handle, 1, 0, 1)
        send_motor_command(handle, 2, 0, 1)
        print("\n\nMotors stopped")
    
    except KeyboardInterrupt:
        pass
    finally:
        if handle:
            handle.close()
        context.close()

if __name__ == "__main__":
    main()
