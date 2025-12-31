"""
ROS2 Node for LX-16A Servo Bus

Provides interface between ROS2 and the LX-16A serial bus servos.
Publishes joint states and subscribes to joint commands.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .lx16a import LX16ADriver, SawppyServos


class ServoBusNode(Node):
    """ROS2 node for controlling LX-16A servos."""

    # Joint names in order matching servo IDs
    DRIVE_JOINTS = [
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'mid_left_wheel_joint',
        'mid_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint',
    ]

    STEER_JOINTS = [
        'front_left_steer_joint',
        'front_right_steer_joint',
        'rear_left_steer_joint',
        'rear_right_steer_joint',
    ]

    def __init__(self):
        super().__init__('servo_bus_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('read_positions', True)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.read_positions = self.get_parameter('read_positions').value

        # Initialize driver
        self.driver = LX16ADriver(port=port, baudrate=baudrate)
        if not self.driver.open():
            self.get_logger().error(f'Failed to open serial port {port}')
            raise RuntimeError(f'Failed to open serial port {port}')

        self.get_logger().info(f'Opened serial port {port} at {baudrate} baud')

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Subscribers
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.drive_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'drive_velocity_cmd',
            self.drive_cmd_callback,
            qos
        )

        self.steer_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'steer_position_cmd',
            self.steer_cmd_callback,
            qos
        )

        # Timer for publishing joint states
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.timer_callback)

        # State
        self.drive_velocities = [0.0] * 6
        self.steer_angles = [0.0] * 4
        self.steer_positions = [500] * 4  # Raw positions

        self.get_logger().info('Servo bus node initialized')

    def drive_cmd_callback(self, msg: Float64MultiArray):
        """Handle drive velocity commands."""
        if len(msg.data) < 6:
            self.get_logger().warning('Drive command needs 6 values')
            return

        self.drive_velocities = list(msg.data[:6])

        # Send to servos (velocity in percent -100 to 100)
        for i, servo_id in enumerate(SawppyServos.ALL_DRIVE):
            velocity = self.drive_velocities[i]
            self.driver.spin(servo_id, velocity)

    def steer_cmd_callback(self, msg: Float64MultiArray):
        """Handle steering position commands."""
        if len(msg.data) < 4:
            self.get_logger().warning('Steer command needs 4 values')
            return

        self.steer_angles = list(msg.data[:4])

        # Send to servos (angle in degrees -120 to 120)
        for i, servo_id in enumerate(SawppyServos.ALL_STEER):
            angle = self.steer_angles[i]
            self.driver.move_to(servo_id, angle)

    def timer_callback(self):
        """Publish joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Build joint names
        msg.name = self.DRIVE_JOINTS + self.STEER_JOINTS

        # Read positions if enabled
        if self.read_positions:
            # Read steering positions
            for i, servo_id in enumerate(SawppyServos.ALL_STEER):
                pos = self.driver.read_position(servo_id)
                if pos is not None:
                    self.steer_positions[i] = pos
                    self.steer_angles[i] = (pos - 500) * (120.0 / 500.0)

        # Convert to radians for joint state
        # Drive joints - report velocity (rad/s estimated from command)
        # For continuous joints, position wraps but we report velocity
        drive_positions = [0.0] * 6  # Position tracking not implemented
        drive_velocities = [v * 0.01 * 2 * math.pi for v in self.drive_velocities]

        # Steer joints - report angle in radians
        steer_positions_rad = [math.radians(a) for a in self.steer_angles]

        msg.position = drive_positions + steer_positions_rad
        msg.velocity = drive_velocities + [0.0] * 4
        msg.effort = []

        self.joint_state_pub.publish(msg)

    def shutdown(self):
        """Shutdown node and stop all servos."""
        self.get_logger().info('Shutting down, stopping all servos...')

        # Stop all drive motors
        for servo_id in SawppyServos.ALL_DRIVE:
            self.driver.spin(servo_id, 0)
            self.driver.set_servo_mode(servo_id)

        # Center all steering
        for servo_id in SawppyServos.ALL_STEER:
            self.driver.move_to(servo_id, 0)

        self.driver.close()


def main(args=None):
    rclpy.init(args=args)

    node = ServoBusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
