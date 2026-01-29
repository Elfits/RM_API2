"""
DaHuan (DH) Two-Finger Gripper Control Module

Copyright (c) 2025 Zhihao Liu, KTH Royal Institute of Technology, Sweden.
Author: Zhihao Liu
Email: zhihaoliu@kth.se

Description:
    Control DH two-finger gripper via Modbus RTU protocol.
    Supports force and position control for left/right hand grippers.

License: MIT
"""

import socket
import time
import json


class DH_Gripper:
    """DH Two-Finger Gripper Control Class"""

    # Modbus register addresses
    REG_INIT = 256      # Initialization
    REG_FORCE = 257     # Force (0-100%)
    REG_POSITION = 259  # Position (0-1000)

    def __init__(self, ip='192.168.1.33', port=8082):
        self.ip = ip
        self.port = port
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((ip, port))
        print(f"Connected to {ip}:{port}")

    def _send_cmd(self, cmd: dict):
        """Send JSON command"""
        cmd_str = json.dumps(cmd) + '\r\n'
        self.client.send(cmd_str.encode('utf-8'))
        time.sleep(0.1)
        # Optional: receive response
        # return self.client.recv(1024).decode()

    def _write_register(self, address: int, data: int):
        """Write single Modbus register"""
        cmd = {
            "command": "write_single_register",
            "port": 1,
            "address": address,
            "data": data,
            "device": 1
        }
        self._send_cmd(cmd)

    def setup(self):
        """Initialize gripper: power on + configure communication + initialize"""
        # Set tool end 24V power supply
        self._send_cmd({"command": "set_tool_voltage", "voltage_type": 3})
        print("Set tool voltage output 24V")
        time.sleep(1)

        # Configure Modbus RTU
        self._send_cmd({
            "command": "set_modbus_mode",
            "port": 1,
            "baudrate": 115200,
            "timeout": 2
        })
        print("Configure communication port ModbusRTU mode")
        time.sleep(1)

        # Execute initialization
        self._write_register(self.REG_INIT, 1)
        print("Initialization successful")
        time.sleep(2)

    def set_force(self, force_percent: int):
        """Set gripping force (0-100%)"""
        force_percent = max(0, min(100, force_percent))
        self._write_register(self.REG_FORCE, force_percent)
        print(f"Force set: {force_percent}%")

    def set_position(self, position: int):
        """Set position (0=closed, 1000=fully open)"""
        position = max(0, min(1000, position))
        self._write_register(self.REG_POSITION, position)
        print(f"Position set: {position}")

    def grasp(self, position: int, force_percent: int, wait_time: float = 1.0):
        """Set force and position simultaneously for grasping

        Args:
            position: Target position (0=closed, 1000=fully open)
            force_percent: Gripping force percentage (0-100%)
            wait_time: Time to wait for action completion (seconds)
        """
        self.set_force(force_percent)
        time.sleep(0.2)
        self.set_position(position)
        time.sleep(wait_time)

    def open(self, force_percent: int = 50):
        """Fully open gripper"""
        self.grasp(position=1000, force_percent=force_percent)
        print("Gripper fully open")

    def close_gripper(self, force_percent: int = 30):
        """Close gripper"""
        self.grasp(position=0, force_percent=force_percent)
        print("Gripper closed")

    def close(self):
        """Close connection"""
        self.client.close()
        print("Connection closed")


# Single gripper test
def test_single_gripper():
    gripper = DH_Gripper(ip='192.168.1.33', port=8082)
    gripper.setup()

    try:
        while True:
            gripper.set_position(0)
            print("###gripper close###")
            time.sleep(3)

            gripper.set_position(500)
            print("###gripper half open###")
            time.sleep(3)

            gripper.set_position(0)
            print("###gripper close###")
            time.sleep(3)

            gripper.set_position(1000)
            print("###gripper full open###")
            time.sleep(3)

    except KeyboardInterrupt:
        print("\nProgram terminated")
    finally:
        gripper.close()


# Left and right hand test
def test_dual_gripper():
    """Left and right gripper synchronization test"""
    IP = '192.168.1.33'
    LEFT_PORT = 8082
    RIGHT_PORT = 8083

    print("=" * 50)
    print("Left and Right Gripper Test")
    print("=" * 50)

    # Initialize left and right grippers
    print("\n[Initialize left gripper]")
    left = DH_Gripper(ip=IP, port=LEFT_PORT)
    left.setup()

    print("\n[Initialize right gripper]")
    right = DH_Gripper(ip=IP, port=RIGHT_PORT)
    right.setup()

    try:
        while True:
            # Test 1: Both hands close
            print("\n--- Both close ---")
            left.set_position(0)
            right.set_position(0)
            time.sleep(3)

            # Test 2: Both hands fully open
            print("\n--- Both fully open ---")
            left.set_position(1000)
            right.set_position(1000)
            time.sleep(3)

            # Test 3: Left open, right close
            print("\n--- Left open, right close ---")
            left.set_position(1000)
            right.set_position(0)
            time.sleep(3)

            # Test 4: Left close, right open
            print("\n--- Left close, right open ---")
            left.set_position(0)
            right.set_position(1000)
            time.sleep(3)

            # Test 5: Alternating motion
            print("\n--- Alternating motion ---")
            for _ in range(3):
                left.set_position(0)
                right.set_position(1000)
                time.sleep(1)
                left.set_position(1000)
                right.set_position(0)
                time.sleep(1)

            # Test 6: Synchronized gradual open/close
            print("\n--- Synchronized gradual open/close ---")
            for pos in range(0, 1001, 200):
                print(f"Position: {pos}")
                left.set_position(pos)
                right.set_position(pos)
                time.sleep(0.5)

            for pos in range(1000, -1, -200):
                print(f"Position: {pos}")
                left.set_position(pos)
                right.set_position(pos)
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nProgram terminated")
    finally:
        left.close()
        right.close()
        print("Left and right connections closed")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == 'dual':
        test_dual_gripper()
    else:
        print("Usage:")
        print("  python dh.py        - Single gripper test")
        print("  python dh.py dual   - Left and right gripper test")
        print()
        test_single_gripper()
