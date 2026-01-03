#!/usr/bin/env python3
"""
ROS2 IMU Publisher Node for Sawppy

Publishes IMU data from BNO055 9-axis IMU (accelerometer, gyroscope, magnetometer).
Unlike simpler 6-axis IMUs, BNO055 provides fused orientation as quaternion.

Publications:
    /imu/data (sensor_msgs/Imu) - IMU readings at configured rate

Parameters:
    simulate (bool): Run without hardware (default: false)
    publish_rate (float): Publishing frequency in Hz (default: 50.0)
    frame_id (str): TF frame ID for IMU (default: 'imu_link')
    i2c_bus (int): I2C bus number (default: 1, Pi 5 also has bus 3)
    i2c_address (int): I2C address of BNO055 (default: 0x28, alt: 0x29)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Hardware imports with fallback
try:
    import adafruit_bno055
    HAS_BNO055 = True
except ImportError:
    HAS_BNO055 = False

try:
    # Try extended bus first (allows specifying bus number)
    from adafruit_extended_bus import ExtendedI2C
    HAS_EXTENDED_BUS = True
except ImportError:
    HAS_EXTENDED_BUS = False

try:
    # Fallback to standard Blinka
    import board
    import busio
    HAS_BLINKA = True
except ImportError:
    HAS_BLINKA = False

HAS_HARDWARE = HAS_BNO055 and (HAS_EXTENDED_BUS or HAS_BLINKA)


class IMUPublisherNode(Node):
    """ROS2 node for BNO055 IMU publishing."""

    def __init__(self):
        super().__init__('imu_publisher')

        # Declare parameters
        self.declare_parameter('simulate', False)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x28)

        # Get parameters
        self.simulate = self.get_parameter('simulate').value or not HAS_HARDWARE
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value

        # Initialize hardware
        self.imu = None
        if not self.simulate:
            try:
                i2c = self._create_i2c_bus(self.i2c_bus)
                self.imu = adafruit_bno055.BNO055_I2C(i2c, address=self.i2c_address)
                # Wait for calibration status
                self.get_logger().info(
                    f'BNO055 initialized on bus {self.i2c_bus} at 0x{self.i2c_address:02X}'
                )
                self.get_logger().info(f'Calibration status: sys={self.imu.calibration_status[0]}, '
                                       f'gyro={self.imu.calibration_status[1]}, '
                                       f'accel={self.imu.calibration_status[2]}, '
                                       f'mag={self.imu.calibration_status[3]}')
            except Exception as e:
                self.get_logger().error(f'BNO055 init failed: {e}')
                self.simulate = True

        if self.simulate:
            self.get_logger().info('Running in simulation mode')

        # Publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu)

        self.get_logger().info(f'IMU publisher ready at {self.publish_rate} Hz')

    def _create_i2c_bus(self, bus_num: int):
        """
        Create I2C bus object for the specified bus number.

        Args:
            bus_num: I2C bus number (1 is default on Pi, 3 also available on Pi 5)

        Returns:
            I2C bus object compatible with adafruit libraries
        """
        # Prefer ExtendedI2C if available (supports any bus number)
        if HAS_EXTENDED_BUS:
            self.get_logger().info(f'Using ExtendedI2C for bus {bus_num}')
            return ExtendedI2C(bus_num)

        # Fall back to standard Blinka (only supports default bus)
        if HAS_BLINKA:
            if bus_num != 1:
                self.get_logger().warning(
                    f'Bus {bus_num} requested but only bus 1 supported without '
                    'adafruit-extended-bus package. Falling back to bus 1.'
                )
            self.get_logger().info('Using standard Blinka I2C')
            return busio.I2C(board.SCL, board.SDA)

        raise RuntimeError('No I2C library available')

    def publish_imu(self):
        """Read IMU and publish data."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        if self.simulate:
            # Simulated level reading (identity quaternion, gravity on Z axis)
            msg.orientation.w = 1.0
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = 9.81
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0
        else:
            try:
                # Read quaternion orientation (BNO055 provides fused orientation)
                quat = self.imu.quaternion
                if quat is not None and quat[0] is not None:
                    # BNO055 returns (w, x, y, z) order
                    msg.orientation.w = float(quat[0])
                    msg.orientation.x = float(quat[1])
                    msg.orientation.y = float(quat[2])
                    msg.orientation.z = float(quat[3])
                else:
                    # Default to identity if not ready
                    msg.orientation.w = 1.0
                    msg.orientation.x = 0.0
                    msg.orientation.y = 0.0
                    msg.orientation.z = 0.0

                # Read linear acceleration (m/s^2) - gravity compensated
                accel = self.imu.linear_acceleration
                if accel is not None and accel[0] is not None:
                    msg.linear_acceleration.x = float(accel[0])
                    msg.linear_acceleration.y = float(accel[1])
                    msg.linear_acceleration.z = float(accel[2])
                else:
                    msg.linear_acceleration.z = 0.0

                # Read gyroscope (rad/s)
                gyro = self.imu.gyro
                if gyro is not None and gyro[0] is not None:
                    msg.angular_velocity.x = float(gyro[0])
                    msg.angular_velocity.y = float(gyro[1])
                    msg.angular_velocity.z = float(gyro[2])
                else:
                    msg.angular_velocity.x = 0.0
                    msg.angular_velocity.y = 0.0
                    msg.angular_velocity.z = 0.0

            except Exception as e:
                self.get_logger().warning(f'IMU read error: {e}')
                # Publish identity orientation on error
                msg.orientation.w = 1.0
                msg.orientation.x = 0.0
                msg.orientation.y = 0.0
                msg.orientation.z = 0.0

        # Covariance matrices (diagonal)
        # Orientation covariance (BNO055 provides good orientation)
        orient_cov = 0.01
        msg.orientation_covariance[0] = orient_cov
        msg.orientation_covariance[4] = orient_cov
        msg.orientation_covariance[8] = orient_cov

        # Angular velocity covariance (rad/s)^2
        angular_cov = 0.01
        msg.angular_velocity_covariance[0] = angular_cov
        msg.angular_velocity_covariance[4] = angular_cov
        msg.angular_velocity_covariance[8] = angular_cov

        # Linear acceleration covariance (m/s^2)^2
        linear_cov = 0.1
        msg.linear_acceleration_covariance[0] = linear_cov
        msg.linear_acceleration_covariance[4] = linear_cov
        msg.linear_acceleration_covariance[8] = linear_cov

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
