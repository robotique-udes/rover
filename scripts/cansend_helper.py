#!/usr/bin/env python3
import can
import struct
import time
import threading
import queue
from enum import Enum
import select
import sys
import os

class Colors:
    GREEN = '\033[92m'
    BLUE = '\033[94m'
    END = '\033[0m'

class ValueType(Enum):
    BOOL = 1
    INT8 = 2
    INT16 = 3
    INT32 = 4
    UINT8 = 5
    UINT16 = 6
    UINT32 = 7
    FLOAT = 8

def success_print(text):
    print(f"{Colors.GREEN}{text}{Colors.END}")

def config_print(text):
    print(f"{Colors.BLUE}{text}{Colors.END}")

def get_data_format(value_type):
    format_map = {
        ValueType.BOOL: '?',
        ValueType.INT8: 'b',
        ValueType.INT16: '<h',
        ValueType.INT32: '<i',
        ValueType.UINT8: 'B',
        ValueType.UINT16: '<H',
        ValueType.UINT32: '<I',
        ValueType.FLOAT: '<f',
    }
    return format_map.get(value_type, 'B')

def parse_value(value_str, value_type):
    if value_type == ValueType.BOOL:
        return value_str.lower() in ('true', '1', 'yes', 'y')
    elif value_type == ValueType.FLOAT:
        return float(value_str)
    else:
        return int(value_str, 16) if value_str.lower().startswith('0x') else int(value_str)

def pack_message(msg_id, msg_content_id, value_type, value):
    data = bytearray([msg_id, msg_content_id])
    format_char = get_data_format(value_type)
    packed_value = struct.pack(format_char, value)
    data.extend(packed_value)
    
    return data[:8] if len(data) > 8 else data

def input_thread_function(value_queue, value_type):
    print(f"\nEnter new {value_type.name} value (or 'q' to quit): ", end='', flush=True)
    
    while True:
        try:
            if sys.platform == 'win32':
                new_value_str = input().strip()
            else:
                i, o, e = select.select([sys.stdin], [], [], 0.1)
                if i:
                    new_value_str = sys.stdin.readline().strip()
                else:
                    continue
            
            if new_value_str.lower() == 'q':
                value_queue.put(None)
                break
            
            new_value = parse_value(new_value_str, value_type)
            value_queue.put(new_value)
            
            if value_type in (ValueType.INT8, ValueType.INT16, ValueType.INT32, 
                             ValueType.UINT8, ValueType.UINT16, ValueType.UINT32):
                success_print(f"Value updated to: {new_value} (0x{new_value:X})")
            else:
                success_print(f"Value updated to: {new_value}")
            
            print(f"\nEnter new {value_type.name} value (or 'q' to quit): ", end='', flush=True)
                
        except ValueError as e:
            print(f"Invalid value: {e}. Try again.")
            print(f"\nEnter new {value_type.name} value (or 'q' to quit): ", end='', flush=True)
        except Exception as e:
            print(f"Error: {e}")
            print(f"\nEnter new {value_type.name} value (or 'q' to quit): ", end='', flush=True)

def send_can_messages(bus, can_id, msg_id, msg_content_id, value_type, initial_value, frequency, value_queue):
    current_value = initial_value
    data = pack_message(msg_id, msg_content_id, value_type, current_value)
    
    try:
        while True:
            try:
                new_value = value_queue.get_nowait()
                if new_value is None:
                    break
                current_value = new_value
                data = pack_message(msg_id, msg_content_id, value_type, current_value)
            except queue.Empty:
                pass
            
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=False
            )
            
            bus.send(msg)
            time.sleep(1.0/frequency)
            
    except KeyboardInterrupt:
        print("\nSending stopped by user")
    except Exception as e:
        print(f"\nError sending message: {e}")

def display_device_ids():
    device_categories = {
        "MASTER": [
            ("0x020", "MASTER_COMPUTER_UNIT"),
            ("0x021", "BATTERY"),
            ("0x022", "PDB_CONTROLLER"),
            ("0x023", "DDB_CONTROLLER")
        ],
        "PROPULSION": [
            ("0x101", "FRONTLEFT_MOTOR"),
            ("0x102", "FRONTRIGHT_MOTOR"),
            ("0x105", "REARLEFT_MOTOR"),
            ("0x106", "REARRIGHT_MOTOR")
        ],
        "ARM": [
            ("0x201", "JL_CONTROLLER"),
            ("0x202", "JR_CONTROLLER"),
            ("0x203", "J1_CONTROLLER"),
            ("0x204", "J2_CONTROLLER"),
            ("0x205", "GRIPPER_TILT_CONTROLLER"),
            ("0x206", "GRIPPER_ROT_CONTROLLER"),
            ("0x207", "GRIPPER_CLOSE_CONTROLLER"),
            ("0x208", "GRIPPER_LASER"),
            ("0x209", "GRIPPER_DISTANCE")
        ],
        "GREEN AUXILIARY": [
            ("0x301", "GPS"),
            ("0x302", "GNSS"),
            ("0x303", "COMPASS"),
            ("0x304", "LIGHTS_MAIN"),
            ("0x305", "LIGHTS_IR")
        ],
        "FREE AUXILIARY": [
            ("0x401", "CAMERA_ROVER_FPV"),
            ("0x402", "CAMERA_ROVER_ANTENNA"),
            ("0x403", "CAMERA_ROVER_FRONT"),
            ("0x404", "CAMERA_ROVER_SCIENCE"),
            ("0x405", "CAMERA_ARM_CENTER"),
            ("0x406", "CAMERA_ARM_SIDE"),
            ("0x407", "SPEAKERS")
        ],
        "INTERNAL": [
            ("0x7FF", "TEST_DEVICE")
        ]
    }
    
    print("\nAvailable Device IDs:")
    print("====================")
    print("Category         | ID    | Device Name")
    print("----------------|-------|-------------------------")
    
    for category, devices in device_categories.items():
        for i, (device_id, device_name) in enumerate(devices):
            if i == 0:
                print(f"{category.ljust(16)}| {device_id} | {device_name}")
            else:
                print(f"               | {device_id} | {device_name}")
    print()

def display_message_ids():
    messages = [
        ("0x01", "TEST_MSG"),
        ("0x02", "TEST_MSG_2"),
        ("0x11", "ERROR_STATE"),
        ("0x12", "HEARTBEAT"),
        ("0x13", "POWER_CMD"),
        ("0x14", "POWER_STATUS"),
        ("0x15", "PWM_CMD"),
        ("0x16", "PWM_STATUS"),
        ("0x17", "PWM_INFO"),
        ("0x18", "PROP_SPEED_CMD"),
        ("0x19", "PROP_SPEED_STATUS"),
        ("0x1A", "ARM_SPEED_CMD"),
        ("0x1B", "ARM_POSITION_STATUS"),
        ("0x1C", "ARM_JOINT_CONFIG"),
        ("0x1D", "FIX_POSITION"),
        ("0x1E", "FIX_HEADING"),
        ("0x1F", "FIX_INFO"),
        ("0x20", "CAM_POSITION_CMD"),
        ("0x21", "CAM_POSITION_STATUS"),
        ("0x22", "DDB_CMD"),
        ("0x23", "DDB_STATUS")
    ]
    
    print("Available Message IDs:")
    print("====================")
    print("ID    | Message Name")
    print("------|------------------")
    for msg_id, msg_name in messages:
        print(f"{msg_id}  | {msg_name}")
    print()

def parse_hex_input(input_text, input_name):
    if input_text.lower().startswith('0x'):
        input_text = input_text[2:]
    try:
        return int(input_text, 16)
    except ValueError:
        raise ValueError(f"Invalid hex value for {input_name}: '{input_text}'")

def main():
    print("CAN Bus Message Sender")
    print("=====================")
    config_print("Using socketcan interface on canRovus channel")
    
    interface = "socketcan"
    channel = "canRovus"
    
    try:
        frequency = float(input("Enter send frequency (Hz): ").strip())
        
        display_device_ids()
        can_id = parse_hex_input(input("Enter CAN ID in hex (with or without 0x prefix): ").strip(), "CAN ID")
        
        display_message_ids()
        msg_id = parse_hex_input(input("Enter message ID in hex (with or without 0x prefix): ").strip(), "message ID")
        msg_content_id = parse_hex_input(input("Enter message content ID in hex (with or without 0x prefix): ").strip(), "message content ID")
        
        print("\nValue types:")
        for t in ValueType:
            print(f"{t.value}. {t.name}")
        
        value_type = ValueType(int(input("\nEnter value type (number): ").strip()))
        value_input = input(f"Enter the initial {value_type.name} value: ").strip()
        initial_value = parse_value(value_input, value_type)
        
        try:
            bus = can.interface.Bus(channel=channel, interface=interface)
            success_print(f"\nConnected to {interface} on {channel}")
        except Exception as e:
            print(f"Error connecting to CAN bus: {e}")
            print("Make sure the interface exists and you have the right permissions.")
            return
        
        value_queue = queue.Queue()
        
        config_print("\nConfiguration complete. Starting message transmission:")
        config_print(f"  CAN ID: 0x{can_id:X}")
        config_print(f"  Message ID: 0x{msg_id:X}")
        config_print(f"  Content ID: 0x{msg_content_id:X}")
        config_print(f"  Value type: {value_type.name}")
        
        if value_type in (ValueType.INT8, ValueType.INT16, ValueType.INT32, 
                         ValueType.UINT8, ValueType.UINT16, ValueType.UINT32):
            config_print(f"  Initial value: {initial_value} (0x{initial_value:X})")
        else:
            config_print(f"  Initial value: {initial_value}")
            
        config_print(f"  Frequency: {frequency} Hz")
        
        input_thread = threading.Thread(
            target=input_thread_function, 
            args=(value_queue, value_type),
            daemon=True
        )
        input_thread.start()
        
        send_can_messages(bus, can_id, msg_id, msg_content_id, value_type, initial_value, frequency, value_queue)
        
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except ValueError as e:
        print(f"\nError: {e}")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        try:
            bus.shutdown()
            print("\nCAN bus connection closed")
        except:
            pass

if __name__ == "__main__":
    main()
