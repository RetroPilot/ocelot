#!/usr/bin/env python3

import usb1
import json
import time

def find_device(vid, pid):
    """Find USB device by VID/PID"""
    context = usb1.USBContext()
    for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == vid and device.getProductID() == pid:
            return device.open()
    return None

def send_json_rpc(device, method, params=None):
    """Send JSON RPC command and get response"""
    request = {
        "jsonrpc": "2.0",
        "method": method,
        "id": 1
    }
    if params:
        request["params"] = params
    
    json_str = json.dumps(request)
    
    # Send command via control transfer 0xFC
    device.controlWrite(0x40, 0xFC, 0, 0, json_str.encode('utf-8'))
    
    time.sleep(0.01)
    
    # Read response via control transfer 0xFB in chunks
    full_response = b''
    offset = 0
    chunk_size = 64
    
    while True:
        chunk = device.controlRead(0xC0, 0xFB, offset, 0, chunk_size)
        if not chunk or len(chunk) == 0:
            break
        full_response += bytes(chunk)
        if len(chunk) < chunk_size:
            break
        offset += len(chunk)
    
    if full_response:
        response_str = full_response.decode('utf-8').rstrip('\x00')
        return json.loads(response_str)
    return None