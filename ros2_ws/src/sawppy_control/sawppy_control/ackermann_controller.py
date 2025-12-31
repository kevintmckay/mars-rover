"""
Ackermann Controller for Sawppy Rover

Converts Twist messages (cmd_vel) to individual wheel speeds and steering angles.

Reference: https://en.wikipedia.org/wiki/Ackermann_steering_geometry
Ported from: sawppy/upstream/esp32_sawppy/lib/rover/wheel_ackermann.c
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class WheelPosition:
    """Position of a wheel relative to rover center."""
    def __init__(self, x: float, y: float):
        self.x = x  # Forward is positive
        self.y = y  # Left is positive


# Chassis dimensions (from wheel_parameter.h)
WHEELBASE_FRONT = 0.14  # meters
WHEELBASE_BACK = 0.14
TRACK_FRONT = 0.20
TRACK_MID = 0.27
TRACK_BACK = 0.20

# Wheel positions relative to rover center
# Order: FL, FR, ML, MR, RL, RR
WHEEL_POSITIONS = [
    WheelPosition(WHEELBASE_FRONT, TRACK_FRONT / 2),   # Front Left
    WheelPosition(WHEELBASE_FRONT, -TRACK_FRONT / 2),  # Front Right
    WheelPosition(0, TRACK_MID / 2),                   # Mid Left
    WheelPosition(0, -TRACK_MID / 2),                  # Mid Right
    WheelPosition(-WHEELBASE_BACK, TRACK_BACK / 2),    # Rear Left
    WheelPosition(-WHEELBASE_BACK, -TRACK_BACK / 2),   # Rear Right
]

# Wheel indices
FL, FR, ML, MR, RL, RR = range(6)

# Steering indices (only corners steer)
STEER_FL, STEER_FR, STEER_RL, STEER_RR = 0, 1, 2, 3

# Max velocity
VELOCITY_LINEAR_MAX = 0.5  # m/s
VELOCITY_ANGULAR_MAX = VELOCITY_LINEAR_MAX * 2 / TRACK_MID  # rad/s

# Wheel radius for speed conversion
WHEEL_RADIUS = 0.06  # meters


class AckermannController(Node):
    """ROS2 node for Ackermann steering control."""

    def __init__(self):
        super().__init__('ackermann_controller')

        # Parameters
        self.declare_parameter('max_linear_velocity', VELOCITY_LINEAR_MAX)
        self.declare_parameter('max_angular_velocity', VELOCITY_ANGULAR_MAX)
        self.declare_parameter('wheel_radius', WHEEL_RADIUS)
        self.declare_parameter('publish_rate', 20.0)

        self.max_linear = self.get_parameter('max_linear_velocity').value
        self.max_angular = self.get_parameter('max_angular_velocity').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        self.drive_pub = self.create_publisher(
            Float64MultiArray,
            'drive_velocity_cmd',
            10
        )
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            'steer_position_cmd',
            10
        )

        # Subscriber
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos
        )

        # State
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.wheel_speeds = [0.0] * 6  # m/s at wheel
        self.steer_angles = [0.0] * 4  # degrees for corners

        # Timer for publishing
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_commands)

        self.get_logger().info(
            f'Ackermann controller initialized. '
            f'Max linear: {self.max_linear:.2f} m/s, '
            f'Max angular: {self.max_angular:.2f} rad/s'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Process incoming velocity command."""
        # Clamp velocities to limits
        linear_x = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular_z = max(-self.max_angular, min(self.max_angular, msg.angular.z))

        # Warn if unsupported axes have values
        if msg.linear.y != 0 or msg.linear.z != 0 or \
           msg.angular.x != 0 or msg.angular.y != 0:
            self.get_logger().warning(
                'Velocity on unsupported axes ignored',
                throttle_duration_sec=5.0
            )

        self.last_linear = linear_x
        self.last_angular = angular_z

        # Calculate wheel speeds and steering angles
        self.calculate_ackermann(linear_x, angular_z)

    def calculate_ackermann(self, linear: float, angular: float):
        """
        Calculate Ackermann steering geometry.

        Args:
            linear: Linear velocity (m/s) - positive is forward
            angular: Angular velocity (rad/s) - positive is CCW (left turn)
        """
        # Reset
        speeds = [0.0] * 6
        steers = [0.0] * 4

        if abs(angular) < 0.001:
            # Straight motion - no steering, all wheels same speed
            for i in range(6):
                speeds[i] = linear
        else:
            # Turning - calculate turn center
            # Turn center is distance from robot center to instantaneous center of rotation
            # Positive turn_center = center is to the left (CCW turn)
            turn_center = linear / angular

            for i in range(6):
                wp = WHEEL_POSITIONS[i]

                # Triangle from wheel to turn center
                opposite = wp.x  # Distance along X axis
                adjacent = turn_center - wp.y  # Distance from wheel to turn center along Y

                # Hypotenuse is distance from wheel to turn center
                hypotenuse = math.sqrt(opposite**2 + adjacent**2)

                # Steering angle (only for corner wheels)
                if abs(opposite) > 0.001:
                    steer_rad = math.atan(opposite / adjacent)
                else:
                    steer_rad = 0.0

                # Wheel speed is proportional to distance from turn center
                speed = angular * hypotenuse

                # Fix sign when not turning in place
                if abs(turn_center) > 0.001:
                    speed = math.copysign(speed, linear)

                # If turn center is between robot center and wheel, reverse direction
                if (turn_center > 0 and wp.y > 0 and wp.y > turn_center) or \
                   (turn_center < 0 and wp.y < 0 and wp.y < turn_center):
                    speed *= -1

                speeds[i] = speed

                # Map wheel index to steering index
                if i == FL:
                    steers[STEER_FL] = math.degrees(steer_rad)
                elif i == FR:
                    steers[STEER_FR] = math.degrees(steer_rad)
                elif i == RL:
                    steers[STEER_RL] = math.degrees(steer_rad)
                elif i == RR:
                    steers[STEER_RR] = math.degrees(steer_rad)

        self.wheel_speeds = speeds
        self.steer_angles = steers

    def publish_commands(self):
        """Publish wheel speeds and steering angles."""
        # Convert wheel speeds (m/s) to motor velocity percentage
        # Assuming max wheel speed corresponds to 100% motor power
        max_wheel_speed = self.max_linear
        drive_cmd = Float64MultiArray()
        drive_cmd.data = [
            (speed / max_wheel_speed) * 100.0
            for speed in self.wheel_speeds
        ]
        self.drive_pub.publish(drive_cmd)

        # Publish steering angles (already in degrees)
        steer_cmd = Float64MultiArray()
        steer_cmd.data = self.steer_angles
        self.steer_pub.publish(steer_cmd)


def main(args=None):
    rclpy.init(args=args)

    node = AckermannController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
