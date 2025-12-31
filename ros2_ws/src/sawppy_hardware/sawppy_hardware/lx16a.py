"""
LX-16A Serial Bus Servo Driver

Python port of the LewanSoul/Hiwonder LX-16A protocol from:
  sawppy/upstream/arduino_sawppy/lewansoul.cpp

Protocol:
  - 115200 baud, 8N1
  - Frame: [0x55, 0x55, ID, Length, Command, Data..., Checksum]
  - Checksum: ~(sum of bytes from ID to end of data) & 0xFF
"""

import serial
import struct
import time
from typing import Optional
from dataclasses import dataclass


# Frame header
FRAME_HEADER = 0x55

# Commands
CMD_MOVE_TIME_WRITE = 1
CMD_MOVE_TIME_READ = 2
CMD_MOVE_TIME_WAIT_WRITE = 7
CMD_MOVE_TIME_WAIT_READ = 8
CMD_MOVE_START = 11
CMD_MOVE_STOP = 12
CMD_ID_WRITE = 13
CMD_ID_READ = 14
CMD_ANGLE_OFFSET_ADJUST = 17
CMD_ANGLE_OFFSET_WRITE = 18
CMD_ANGLE_OFFSET_READ = 19
CMD_ANGLE_LIMIT_WRITE = 20
CMD_ANGLE_LIMIT_READ = 21
CMD_VIN_LIMIT_WRITE = 22
CMD_VIN_LIMIT_READ = 23
CMD_TEMP_MAX_LIMIT_WRITE = 24
CMD_TEMP_MAX_LIMIT_READ = 25
CMD_TEMP_READ = 26
CMD_VIN_READ = 27
CMD_POS_READ = 28
CMD_MODE_WRITE = 29
CMD_MODE_READ = 30
CMD_LOAD_WRITE = 31
CMD_LOAD_READ = 32
CMD_LED_CTRL_WRITE = 33
CMD_LED_CTRL_READ = 34
CMD_LED_ERROR_WRITE = 35
CMD_LED_ERROR_READ = 36

# Servo position range
POS_MIN = 0
POS_MAX = 1000
POS_CENTER = 500

# Angle range (degrees)
ANGLE_MIN = -120.0
ANGLE_MAX = 120.0

# Motor speed range
SPEED_MIN = -1000
SPEED_MAX = 1000


@dataclass
class ServoStatus:
    """Status of an LX-16A servo."""
    id: int
    position: int  # 0-1000
    angle: float   # -120 to 120 degrees
    voltage: float  # Volts
    temperature: int  # Celsius
    mode: int  # 0=servo, 1=motor
    loaded: bool


class LX16ADriver:
    """Driver for LX-16A serial bus servos."""

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 115200,
        timeout: float = 0.1
    ):
        """
        Initialize the LX-16A driver.

        Args:
            port: Serial port path
            baudrate: Baud rate (default 115200)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None

    def open(self) -> bool:
        """Open serial connection."""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            return True
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            return False

    def close(self):
        """Close serial connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @staticmethod
    def _checksum(data: bytes) -> int:
        """Calculate LX-16A checksum."""
        return (~sum(data[2:])) & 0xFF

    def _send_command(self, servo_id: int, cmd: int, data: bytes = b'') -> bool:
        """
        Send a command to a servo.

        Args:
            servo_id: Servo ID (1-253, 254=broadcast)
            cmd: Command byte
            data: Optional data bytes

        Returns:
            True if sent successfully
        """
        if not self._serial or not self._serial.is_open:
            return False

        length = len(data) + 3  # ID + length + cmd + data
        packet = bytes([FRAME_HEADER, FRAME_HEADER, servo_id, length, cmd]) + data
        packet = packet + bytes([self._checksum(packet)])

        try:
            self._serial.write(packet)
            return True
        except serial.SerialException:
            return False

    def _read_response(self, expected_cmd: int) -> Optional[bytes]:
        """
        Read response from servo.

        Args:
            expected_cmd: Expected command in response

        Returns:
            Data bytes from response or None
        """
        if not self._serial or not self._serial.is_open:
            return None

        # Wait for frame header
        header_count = 0
        while header_count < 2:
            byte = self._serial.read(1)
            if not byte:
                return None
            if byte[0] == FRAME_HEADER:
                header_count += 1
            else:
                header_count = 0

        # Read ID and length
        header = self._serial.read(2)
        if len(header) < 2:
            return None

        servo_id = header[0]
        length = header[1]

        # Read command, data, and checksum
        remaining = self._serial.read(length)
        if len(remaining) < length:
            return None

        cmd = remaining[0]
        if cmd != expected_cmd:
            return None

        # Verify checksum
        full_packet = bytes([FRAME_HEADER, FRAME_HEADER, servo_id, length]) + remaining[:-1]
        if remaining[-1] != self._checksum(full_packet):
            return None

        return remaining[1:-1]  # Return data (exclude cmd and checksum)

    # === High-level API ===

    def move_to(self, servo_id: int, angle: float, time_ms: int = 200) -> bool:
        """
        Move servo to angle.

        Args:
            servo_id: Servo ID
            angle: Target angle in degrees (-120 to 120)
            time_ms: Movement time in milliseconds

        Returns:
            True if command sent successfully
        """
        # Clamp angle to valid range
        angle = max(ANGLE_MIN, min(ANGLE_MAX, angle))

        # Convert angle to position (0-1000)
        position = int((angle * (500.0 / 120.0)) + 500)
        position = max(POS_MIN, min(POS_MAX, position))

        # Pack position and time as little-endian uint16
        data = struct.pack('<HH', position, time_ms)
        return self._send_command(servo_id, CMD_MOVE_TIME_WRITE, data)

    def move_to_position(self, servo_id: int, position: int, time_ms: int = 200) -> bool:
        """
        Move servo to raw position.

        Args:
            servo_id: Servo ID
            position: Target position (0-1000)
            time_ms: Movement time in milliseconds

        Returns:
            True if command sent successfully
        """
        position = max(POS_MIN, min(POS_MAX, position))
        data = struct.pack('<HH', position, time_ms)
        return self._send_command(servo_id, CMD_MOVE_TIME_WRITE, data)

    def spin(self, servo_id: int, velocity: float) -> bool:
        """
        Set servo to motor mode and spin at velocity.

        Args:
            servo_id: Servo ID
            velocity: Velocity from -100 to 100 percent

        Returns:
            True if command sent successfully
        """
        # Clamp velocity and convert to -1000 to 1000
        velocity = max(-100, min(100, velocity))
        speed = int(velocity * 10)

        # Mode 1 = motor mode, speed as signed 16-bit
        data = struct.pack('<Bbh', 1, 0, speed)
        return self._send_command(servo_id, CMD_MODE_WRITE, data)

    def stop(self, servo_id: int) -> bool:
        """Stop servo movement."""
        return self._send_command(servo_id, CMD_MOVE_STOP)

    def set_servo_mode(self, servo_id: int) -> bool:
        """Set servo to position control mode."""
        data = struct.pack('<BBH', 0, 0, 0)
        return self._send_command(servo_id, CMD_MODE_WRITE, data)

    def enable_torque(self, servo_id: int) -> bool:
        """Enable servo torque (load)."""
        return self._send_command(servo_id, CMD_LOAD_WRITE, bytes([1]))

    def disable_torque(self, servo_id: int) -> bool:
        """Disable servo torque (unload)."""
        return self._send_command(servo_id, CMD_LOAD_WRITE, bytes([0]))

    def read_position(self, servo_id: int) -> Optional[int]:
        """
        Read current servo position.

        Args:
            servo_id: Servo ID

        Returns:
            Position (0-1000) or None if failed
        """
        if not self._serial:
            return None

        # Flush input buffer
        self._serial.reset_input_buffer()

        if not self._send_command(servo_id, CMD_POS_READ):
            return None

        data = self._read_response(CMD_POS_READ)
        if data and len(data) >= 2:
            return struct.unpack('<h', data[:2])[0]
        return None

    def read_angle(self, servo_id: int) -> Optional[float]:
        """
        Read current servo angle.

        Args:
            servo_id: Servo ID

        Returns:
            Angle in degrees (-120 to 120) or None if failed
        """
        position = self.read_position(servo_id)
        if position is not None:
            return (position - 500) * (120.0 / 500.0)
        return None

    def read_voltage(self, servo_id: int) -> Optional[float]:
        """
        Read servo input voltage.

        Args:
            servo_id: Servo ID

        Returns:
            Voltage in volts or None if failed
        """
        if not self._serial:
            return None

        self._serial.reset_input_buffer()

        if not self._send_command(servo_id, CMD_VIN_READ):
            return None

        data = self._read_response(CMD_VIN_READ)
        if data and len(data) >= 2:
            mv = struct.unpack('<H', data[:2])[0]
            return mv / 1000.0
        return None

    def read_temperature(self, servo_id: int) -> Optional[int]:
        """
        Read servo temperature.

        Args:
            servo_id: Servo ID

        Returns:
            Temperature in Celsius or None if failed
        """
        if not self._serial:
            return None

        self._serial.reset_input_buffer()

        if not self._send_command(servo_id, CMD_TEMP_READ):
            return None

        data = self._read_response(CMD_TEMP_READ)
        if data and len(data) >= 1:
            return data[0]
        return None

    def set_id(self, old_id: int, new_id: int) -> bool:
        """
        Change servo ID.

        Args:
            old_id: Current servo ID
            new_id: New servo ID (1-253)

        Returns:
            True if command sent successfully
        """
        if not (1 <= new_id <= 253):
            return False
        return self._send_command(old_id, CMD_ID_WRITE, bytes([new_id]))

    def set_led(self, servo_id: int, on: bool) -> bool:
        """
        Control servo LED.

        Args:
            servo_id: Servo ID
            on: True to turn LED on, False to turn off

        Returns:
            True if command sent successfully
        """
        return self._send_command(servo_id, CMD_LED_CTRL_WRITE, bytes([0 if on else 1]))


# Sawppy-specific servo mapping
class SawppyServos:
    """Servo ID mapping for Sawppy rover."""

    # Drive servos (motor mode)
    DRIVE_FL = 25  # Front Left
    DRIVE_FR = 27  # Front Right
    DRIVE_ML = 21  # Mid Left
    DRIVE_MR = 22  # Mid Right
    DRIVE_RL = 20  # Rear Left
    DRIVE_RR = 28  # Rear Right

    # Steering servos (servo mode) - corners only
    STEER_FL = 23  # Front Left
    STEER_FR = 29  # Front Right
    STEER_RL = 24  # Rear Left
    STEER_RR = 26  # Rear Right

    ALL_DRIVE = [DRIVE_FL, DRIVE_FR, DRIVE_ML, DRIVE_MR, DRIVE_RL, DRIVE_RR]
    ALL_STEER = [STEER_FL, STEER_FR, STEER_RL, STEER_RR]
    ALL = ALL_DRIVE + ALL_STEER


def main():
    """Test the LX-16A driver."""
    import argparse

    parser = argparse.ArgumentParser(description='LX-16A Servo Driver Test')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port')
    parser.add_argument('--id', type=int, default=1, help='Servo ID')
    parser.add_argument('--scan', action='store_true', help='Scan for servos')
    parser.add_argument('--move', type=float, help='Move to angle (-120 to 120)')
    parser.add_argument('--spin', type=float, help='Spin at velocity (-100 to 100)')
    parser.add_argument('--stop', action='store_true', help='Stop servo')
    parser.add_argument('--read', action='store_true', help='Read servo status')
    args = parser.parse_args()

    with LX16ADriver(args.port) as driver:
        if args.scan:
            print("Scanning for servos...")
            found = []
            for servo_id in range(1, 254):
                pos = driver.read_position(servo_id)
                if pos is not None:
                    print(f"  Found servo ID {servo_id}, position: {pos}")
                    found.append(servo_id)
            print(f"Found {len(found)} servos: {found}")

        elif args.move is not None:
            print(f"Moving servo {args.id} to {args.move} degrees")
            driver.move_to(args.id, args.move)

        elif args.spin is not None:
            print(f"Spinning servo {args.id} at {args.spin}%")
            driver.spin(args.id, args.spin)

        elif args.stop:
            print(f"Stopping servo {args.id}")
            driver.stop(args.id)
            driver.set_servo_mode(args.id)

        elif args.read:
            print(f"Reading servo {args.id}...")
            pos = driver.read_position(args.id)
            angle = driver.read_angle(args.id)
            voltage = driver.read_voltage(args.id)
            temp = driver.read_temperature(args.id)
            print(f"  Position: {pos}")
            print(f"  Angle: {angle:.1f}°" if angle else "  Angle: N/A")
            print(f"  Voltage: {voltage:.2f}V" if voltage else "  Voltage: N/A")
            print(f"  Temperature: {temp}°C" if temp else "  Temperature: N/A")

        else:
            parser.print_help()


if __name__ == '__main__':
    main()
