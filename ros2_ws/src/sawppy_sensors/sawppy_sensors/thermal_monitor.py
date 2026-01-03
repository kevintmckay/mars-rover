#!/usr/bin/env python3
"""
ROS2 Thermal Monitor Node for Raspberry Pi 5

Monitors CPU temperature and throttling status, publishing diagnostics
for system health monitoring.

Publications:
    /thermal/status (std_msgs/String) - JSON status for easy parsing
    /diagnostics (diagnostic_msgs/DiagnosticArray) - standard diagnostics

Parameters:
    publish_rate (float): Publishing frequency in Hz (default: 1.0)
    warn_temp (float): Temperature warning threshold in C (default: 70.0)
    critical_temp (float): Temperature critical threshold in C (default: 80.0)
"""

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Thermal zone paths (Pi 5)
THERMAL_ZONE = Path('/sys/class/thermal/thermal_zone0/temp')
THROTTLE_STATUS = Path('/sys/devices/platform/soc/soc:firmware/get_throttled')

# Throttle bit flags (from vcgencmd)
THROTTLE_BITS = {
    0: 'under_voltage',
    1: 'freq_capped',
    2: 'currently_throttled',
    3: 'soft_temp_limit',
    16: 'under_voltage_occurred',
    17: 'freq_capped_occurred',
    18: 'throttled_occurred',
    19: 'soft_temp_limit_occurred',
}


class ThermalMonitorNode(Node):
    """ROS2 node for Raspberry Pi thermal monitoring."""

    def __init__(self):
        super().__init__('thermal_monitor')

        # Declare parameters
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('warn_temp', 70.0)
        self.declare_parameter('critical_temp', 80.0)

        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.warn_temp = self.get_parameter('warn_temp').value
        self.critical_temp = self.get_parameter('critical_temp').value

        # Check if running on Pi
        self.has_thermal = THERMAL_ZONE.exists()
        self.has_throttle = THROTTLE_STATUS.exists()

        if not self.has_thermal:
            self.get_logger().warning('Thermal zone not found - not running on Pi?')

        # Publishers
        self.status_pub = self.create_publisher(String, 'thermal/status', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_thermal)

        self.get_logger().info(
            f'Thermal monitor ready (warn={self.warn_temp}C, critical={self.critical_temp}C)'
        )

    def read_temperature(self) -> float | None:
        """Read CPU temperature in Celsius."""
        if not self.has_thermal:
            return None
        try:
            temp_millic = int(THERMAL_ZONE.read_text().strip())
            return temp_millic / 1000.0
        except (IOError, ValueError) as e:
            self.get_logger().warning(f'Failed to read temperature: {e}')
            return None

    def read_throttle_status(self) -> dict:
        """Read throttle status and decode flags."""
        result = {
            'raw': 0,
            'under_voltage': False,
            'freq_capped': False,
            'throttled': False,
            'soft_temp_limit': False,
            'any_current': False,
            'any_occurred': False,
        }

        if not self.has_throttle:
            return result

        try:
            raw = int(THROTTLE_STATUS.read_text().strip(), 16)
            result['raw'] = raw

            # Decode current flags (bits 0-3)
            result['under_voltage'] = bool(raw & (1 << 0))
            result['freq_capped'] = bool(raw & (1 << 1))
            result['throttled'] = bool(raw & (1 << 2))
            result['soft_temp_limit'] = bool(raw & (1 << 3))

            # Any current issues?
            result['any_current'] = bool(raw & 0x0F)

            # Any issues since boot? (bits 16-19)
            result['any_occurred'] = bool(raw & 0xF0000)

        except (IOError, ValueError) as e:
            self.get_logger().warning(f'Failed to read throttle status: {e}')

        return result

    def publish_thermal(self):
        """Read and publish thermal status."""
        temp = self.read_temperature()
        throttle = self.read_throttle_status()

        # Determine status level
        if temp is None:
            level = DiagnosticStatus.STALE
            status_str = 'unknown'
        elif temp >= self.critical_temp or throttle['throttled']:
            level = DiagnosticStatus.ERROR
            status_str = 'critical'
        elif temp >= self.warn_temp or throttle['any_current']:
            level = DiagnosticStatus.WARN
            status_str = 'warning'
        else:
            level = DiagnosticStatus.OK
            status_str = 'ok'

        # Build status message
        status_msg = String()
        status_data = {
            'status': status_str,
            'temperature_c': temp,
            'throttle': throttle,
        }
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

        # Build diagnostic message
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        diag_status = DiagnosticStatus()
        diag_status.name = 'Pi5 Thermal'
        diag_status.hardware_id = 'raspberry_pi_5'
        diag_status.level = level

        if temp is not None:
            diag_status.message = f'{temp:.1f}C'
            if throttle['throttled']:
                diag_status.message += ' [THROTTLED]'
            elif throttle['freq_capped']:
                diag_status.message += ' [FREQ CAPPED]'
            elif throttle['under_voltage']:
                diag_status.message += ' [UNDER VOLTAGE]'
        else:
            diag_status.message = 'Temperature unavailable'

        # Add key-value pairs
        diag_status.values = [
            KeyValue(key='temperature_c', value=f'{temp:.1f}' if temp else 'N/A'),
            KeyValue(key='warn_threshold', value=f'{self.warn_temp:.1f}'),
            KeyValue(key='critical_threshold', value=f'{self.critical_temp:.1f}'),
            KeyValue(key='throttled', value=str(throttle['throttled'])),
            KeyValue(key='freq_capped', value=str(throttle['freq_capped'])),
            KeyValue(key='under_voltage', value=str(throttle['under_voltage'])),
            KeyValue(key='soft_temp_limit', value=str(throttle['soft_temp_limit'])),
            KeyValue(key='throttle_raw', value=hex(throttle['raw'])),
        ]

        # Log warnings
        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(
                f'THERMAL CRITICAL: {diag_status.message}',
                throttle_duration_sec=10.0
            )
        elif level == DiagnosticStatus.WARN:
            self.get_logger().warning(
                f'Thermal warning: {diag_status.message}',
                throttle_duration_sec=30.0
            )

        diag_msg.status.append(diag_status)
        self.diag_pub.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
