#!/usr/bin/env python3
"""
ROS2 ToF Publisher Node for Sawppy

Publishes range data from VL53L0X Time-of-Flight sensor.
Used as a front bumper sensor for close-range obstacle detection.

Publications:
    /tof/range (sensor_msgs/Range) - distance readings

Parameters:
    simulate (bool): Run without hardware (default: false)
    publish_rate (float): Publishing frequency in Hz (default: 10.0)
    frame_id (str): TF frame ID for sensor (default: 'tof_link')
    min_range (float): Minimum valid range in meters (default: 0.03)
    max_range (float): Maximum valid range in meters (default: 1.2)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# Hardware imports with fallback
try:
    import VL53L0X
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False


class ToFPublisherNode(Node):
    """ROS2 node for VL53L0X ToF sensor publishing."""

    def __init__(self):
        super().__init__('tof_publisher')

        # Declare parameters
        self.declare_parameter('simulate', False)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'tof_link')
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('max_range', 1.2)

        # Get parameters
        self.simulate = self.get_parameter('simulate').value or not HAS_HARDWARE
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        # Initialize hardware
        self.tof = None
        if not self.simulate:
            try:
                self.tof = VL53L0X.VL53L0X()
                self.tof.open()
                self.tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
                self.get_logger().info('VL53L0X initialized at 0x29')
            except Exception as e:
                self.get_logger().error(f'VL53L0X init failed: {e}')
                self.simulate = True

        if self.simulate:
            self.get_logger().info('Running in simulation mode')

        # Publisher
        self.range_pub = self.create_publisher(Range, 'tof/range', 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_range)

        self.get_logger().info(f'ToF publisher ready at {self.publish_rate} Hz')

    def publish_range(self):
        """Read ToF sensor and publish range data."""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.44  # ~25 degree cone
        msg.min_range = self.min_range
        msg.max_range = self.max_range

        if self.simulate:
            # Simulated mid-range reading
            msg.range = 0.5
        else:
            try:
                distance_mm = self.tof.get_distance()
                distance_m = distance_mm / 1000.0

                if distance_m < self.min_range or distance_m > self.max_range:
                    msg.range = self.max_range  # Out of range - use max for Nav2 compatibility
                else:
                    msg.range = distance_m
            except Exception as e:
                self.get_logger().warning(f'ToF read error: {e}')
                # Publish max_range on error to indicate no obstacle detected
                msg.range = self.max_range

        self.range_pub.publish(msg)

    def cleanup(self):
        """Cleanup on shutdown."""
        if self.tof is not None:
            try:
                self.tof.stop_ranging()
                self.tof.close()
                self.get_logger().info('VL53L0X closed')
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ToFPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
