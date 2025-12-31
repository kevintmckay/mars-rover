"""
LX-16A Test Script

Test individual servos or scan the bus for all connected servos.
"""

import sys
import time
from .lx16a import LX16ADriver, SawppyServos


def scan_servos(driver: LX16ADriver):
    """Scan bus for all connected servos."""
    print("\nScanning for servos (this may take a minute)...")
    found = []

    for servo_id in range(1, 254):
        pos = driver.read_position(servo_id)
        if pos is not None:
            voltage = driver.read_voltage(servo_id)
            temp = driver.read_temperature(servo_id)
            print(f"  ID {servo_id:3d}: pos={pos:4d}, "
                  f"voltage={voltage:.2f}V, temp={temp}°C")
            found.append(servo_id)

    print(f"\nFound {len(found)} servos: {found}")
    return found


def test_sawppy_servos(driver: LX16ADriver):
    """Test all Sawppy servos."""
    print("\n=== Testing Sawppy Servos ===")

    print("\n1. Checking steering servos...")
    for name, servo_id in [
        ('Front Left', SawppyServos.STEER_FL),
        ('Front Right', SawppyServos.STEER_FR),
        ('Rear Left', SawppyServos.STEER_RL),
        ('Rear Right', SawppyServos.STEER_RR),
    ]:
        pos = driver.read_position(servo_id)
        if pos is not None:
            angle = (pos - 500) * (120.0 / 500.0)
            print(f"  {name} (ID {servo_id}): {angle:.1f}°")
        else:
            print(f"  {name} (ID {servo_id}): NOT RESPONDING")

    print("\n2. Checking drive servos...")
    for name, servo_id in [
        ('Front Left', SawppyServos.DRIVE_FL),
        ('Front Right', SawppyServos.DRIVE_FR),
        ('Mid Left', SawppyServos.DRIVE_ML),
        ('Mid Right', SawppyServos.DRIVE_MR),
        ('Rear Left', SawppyServos.DRIVE_RL),
        ('Rear Right', SawppyServos.DRIVE_RR),
    ]:
        voltage = driver.read_voltage(servo_id)
        if voltage is not None:
            print(f"  {name} (ID {servo_id}): {voltage:.2f}V")
        else:
            print(f"  {name} (ID {servo_id}): NOT RESPONDING")


def test_steering_sweep(driver: LX16ADriver):
    """Sweep all steering servos through range."""
    print("\n=== Steering Sweep Test ===")
    print("Moving all steering to center...")

    for servo_id in SawppyServos.ALL_STEER:
        driver.move_to(servo_id, 0)
    time.sleep(1)

    for angle in [-30, 0, 30, 0]:
        print(f"Moving to {angle}°...")
        for servo_id in SawppyServos.ALL_STEER:
            driver.move_to(servo_id, angle)
        time.sleep(1)

    print("Steering sweep complete")


def test_drive_brief(driver: LX16ADriver):
    """Brief test of drive motors."""
    print("\n=== Drive Test (Brief) ===")
    print("WARNING: Wheels will spin! Ensure rover is elevated.")
    input("Press Enter to continue or Ctrl+C to abort...")

    print("Spinning forward at 20%...")
    for servo_id in SawppyServos.ALL_DRIVE:
        driver.spin(servo_id, 20)
    time.sleep(1)

    print("Stopping...")
    for servo_id in SawppyServos.ALL_DRIVE:
        driver.spin(servo_id, 0)
        driver.set_servo_mode(servo_id)

    print("Drive test complete")


def main():
    import argparse

    parser = argparse.ArgumentParser(description='LX-16A Sawppy Test')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port')
    parser.add_argument('command', nargs='?', default='status',
                        choices=['scan', 'status', 'steer', 'drive'],
                        help='Test command')
    args = parser.parse_args()

    print(f"Opening {args.port}...")

    with LX16ADriver(args.port) as driver:
        if args.command == 'scan':
            scan_servos(driver)
        elif args.command == 'status':
            test_sawppy_servos(driver)
        elif args.command == 'steer':
            test_steering_sweep(driver)
        elif args.command == 'drive':
            test_drive_brief(driver)


if __name__ == '__main__':
    main()
