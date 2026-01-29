"""
Inspire Dexterous Hand Control Module

Copyright (c) 2025 Zhihao Liu, KTH Royal Institute of Technology, Sweden.
Author: Zhihao Liu
Email: zhihaoliu@kth.se

Description:
    Control Inspire dexterous hand via Modbus TCP protocol.
    Supports 6-finger angle, force, and speed control for left/right hands.

Requirements:
    pip install pymodbus==2.5.3

License: MIT
"""

from pymodbus.client.sync import ModbusTcpClient
import time


class InspireHand:
    """Inspire Dexterous Hand Control Class"""

    # Modbus register addresses
    REG = {
        'ID': 1000,
        'baudrate': 1001,
        'clearErr': 1004,
        'forceClb': 1009,
        'angleSet': 1486,    # Set 6 finger angles (0-1000)
        'forceSet': 1498,    # Set 6 finger forces (0-1000)
        'speedSet': 1522,    # Set 6 finger speeds
        'angleAct': 1546,    # Read actual angles
        'forceAct': 1582,    # Read actual forces
        'errCode': 1606,     # Error codes
        'statusCode': 1612,  # Status codes
        'temp': 1618,        # Temperature
        'actionSeq': 2320,   # Action sequence number
        'actionRun': 2322    # Run action
    }

    def __init__(self, ip='192.168.1.33', port=8082):
        self.ip = ip
        self.port = port
        self.client = ModbusTcpClient(ip, port)
        self.client.connect()
        print(f"Connected to {ip}:{port}")

    def _write_register(self, address: int, value: int):
        """Write single Modbus register"""
        self.client.write_register(address, value)

    def _write_registers(self, address: int, values: list):
        """Write multiple Modbus registers"""
        self.client.write_registers(address, values)

    def _read_registers(self, address: int, count: int):
        """Read multiple Modbus registers"""
        response = self.client.read_holding_registers(address, count)
        return response.registers if not response.isError() else []

    def set_angles(self, angles: list):
        """Set 6 finger angles (0-1000, -1 to skip)

        Args:
            angles: List of 6 values [thumb, index, middle, ring, pinky, thumb_rotate]
        """
        if len(angles) != 6:
            print("Error: angles must be a list of 6 values")
            return
        val_reg = [(a & 0xFFFF) for a in angles]
        self._write_registers(self.REG['angleSet'], val_reg)
        print(f"Angles set: {angles}")

    def set_forces(self, forces: list):
        """Set 6 finger forces (0-1000)

        Args:
            forces: List of 6 values
        """
        if len(forces) != 6:
            print("Error: forces must be a list of 6 values")
            return
        val_reg = [(f & 0xFFFF) for f in forces]
        self._write_registers(self.REG['forceSet'], val_reg)
        print(f"Forces set: {forces}")

    def set_speeds(self, speeds: list):
        """Set 6 finger speeds (0-1000)

        Args:
            speeds: List of 6 values
        """
        if len(speeds) != 6:
            print("Error: speeds must be a list of 6 values")
            return
        val_reg = [(s & 0xFFFF) for s in speeds]
        self._write_registers(self.REG['speedSet'], val_reg)
        print(f"Speeds set: {speeds}")

    def read_angles(self):
        """Read actual finger angles"""
        val = self._read_registers(self.REG['angleAct'], 6)
        if len(val) < 6:
            print("Error: failed to read angles")
            return []
        print(f"Actual angles: {val}")
        return val

    def read_forces(self):
        """Read actual finger forces"""
        val = self._read_registers(self.REG['forceAct'], 6)
        if len(val) < 6:
            print("Error: failed to read forces")
            return []
        print(f"Actual forces: {val}")
        return val

    def read_errors(self):
        """Read error codes"""
        val = self._read_registers(self.REG['errCode'], 3)
        if len(val) < 3:
            print("Error: failed to read error codes")
            return []
        results = []
        for v in val:
            results.append(v & 0xFF)
            results.append((v >> 8) & 0xFF)
        print(f"Error codes: {results}")
        return results

    def read_temps(self):
        """Read temperature"""
        val = self._read_registers(self.REG['temp'], 3)
        if len(val) < 3:
            print("Error: failed to read temperature")
            return []
        results = []
        for v in val:
            results.append(v & 0xFF)
            results.append((v >> 8) & 0xFF)
        print(f"Temperature: {results}")
        return results

    def run_action(self, action_id: int):
        """Run preset action sequence

        Args:
            action_id: Action sequence number
        """
        self._write_registers(self.REG['actionSeq'], [action_id])
        time.sleep(0.1)
        self._write_registers(self.REG['actionRun'], [1])
        print(f"Running action sequence: {action_id}")

    def open_hand(self, speed: int = 100):
        """Open all fingers"""
        self.set_speeds([speed] * 6)
        time.sleep(0.2)
        self.set_angles([0, 0, 0, 0, 0, -1])
        print("Hand open")

    def close_hand(self, force: int = 500, speed: int = 100):
        """Close all fingers (grasp)"""
        self.set_speeds([speed] * 6)
        time.sleep(0.2)
        self.set_forces([force] * 6)
        time.sleep(0.2)
        self.set_angles([1000, 1000, 1000, 1000, 1000, -1])
        print("Hand closed")

    def pinch(self, force: int = 500):
        """Pinch gesture (thumb + index)"""
        self.set_forces([force] * 6)
        time.sleep(0.2)
        self.set_angles([1000, 1000, 0, 0, 0, -1])
        print("Pinch gesture")

    def point(self):
        """Point gesture (index extended)"""
        self.set_angles([1000, 0, 1000, 1000, 1000, -1])
        print("Point gesture")

    def close(self):
        """Close connection"""
        self.client.close()
        print("Connection closed")


# Single hand test
def test_single_hand():
    hand = InspireHand(ip='192.168.1.33', port=8082)

    try:
        print("\n--- Set speed ---")
        hand.set_speeds([100, 100, 100, 100, 100, 100])
        time.sleep(2)

        print("\n--- Set force ---")
        hand.set_forces([500, 500, 500, 500, 500, 500])
        time.sleep(1)

        print("\n--- Open hand (angles=0) ---")
        hand.set_angles([0, 0, 0, 0, 400, -1])
        time.sleep(3)

        hand.read_angles()
        time.sleep(1)

        print("\n--- Close hand (angles=1000) ---")
        hand.set_angles([1000, 1000, 1000, 1000, 1000, -1])
        time.sleep(5)

        hand.read_angles()
        time.sleep(1)

        print("\n--- Error codes ---")
        hand.read_errors()
        time.sleep(1)

        print("\n--- Temperature ---")
        hand.read_temps()
        time.sleep(1)

        print("\n--- Run action sequence 2 ---")
        hand.run_action(2)

    except KeyboardInterrupt:
        print("\nProgram terminated")
    finally:
        hand.close()


# Left and right hand test
def test_dual_hand():
    """Left and right hand synchronization test"""
    IP = '192.168.1.33'
    LEFT_PORT = 8082
    RIGHT_PORT = 8083

    print("=" * 50)
    print("Left and Right Dexterous Hand Test")
    print("=" * 50)

    # Initialize left and right hands
    print("\n[Initialize left hand]")
    left = InspireHand(ip=IP, port=LEFT_PORT)

    print("\n[Initialize right hand]")
    right = InspireHand(ip=IP, port=RIGHT_PORT)

    # Set speed and force for both hands
    for hand in [left, right]:
        hand.set_speeds([100, 100, 100, 100, 100, 100])
        hand.set_forces([500, 500, 500, 500, 500, 500])
    time.sleep(1)

    try:
        while True:
            # Test 1: Both hands open
            print("\n--- Both open ---")
            left.set_angles([0, 0, 0, 0, 0, -1])
            right.set_angles([0, 0, 0, 0, 0, -1])
            time.sleep(3)

            # Test 2: Both hands close
            print("\n--- Both close ---")
            left.set_angles([1000, 1000, 1000, 1000, 1000, -1])
            right.set_angles([1000, 1000, 1000, 1000, 1000, -1])
            time.sleep(3)

            # Test 3: Left open, right close
            print("\n--- Left open, right close ---")
            left.set_angles([0, 0, 0, 0, 0, -1])
            right.set_angles([1000, 1000, 1000, 1000, 1000, -1])
            time.sleep(3)

            # Test 4: Left close, right open
            print("\n--- Left close, right open ---")
            left.set_angles([1000, 1000, 1000, 1000, 1000, -1])
            right.set_angles([0, 0, 0, 0, 0, -1])
            time.sleep(3)

            # Test 5: Both pinch
            print("\n--- Both pinch ---")
            left.pinch()
            right.pinch()
            time.sleep(3)

            # Test 6: Both point
            print("\n--- Both point ---")
            left.point()
            right.point()
            time.sleep(3)

    except KeyboardInterrupt:
        print("\nProgram terminated")
    finally:
        left.close()
        right.close()
        print("Left and right connections closed")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == 'dual':
        test_dual_hand()
    else:
        print("Usage:")
        print("  python inspire.py        - Single hand test")
        print("  python inspire.py dual   - Left and right hand test")
        print()
        test_single_hand()
