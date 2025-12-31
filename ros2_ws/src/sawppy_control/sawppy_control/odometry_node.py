"""
Odometry Node for Sawppy Rover

Estimates robot pose from wheel velocities using Ackermann kinematics.
Publishes odom -> base_footprint transform and Odometry message.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import tf2_ros


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class OdometryNode(Node):
    """ROS2 node for wheel odometry."""

    def __init__(self):
        super().__init__('odometry_node')

        # Parameters
        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )

        # Subscriber for cmd_vel (to estimate velocity)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()

        # Timer
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info('Odometry node initialized')

    def cmd_vel_callback(self, msg: Twist):
        """Update velocity estimate from cmd_vel."""
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def timer_callback(self):
        """Update odometry and publish."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Integrate pose
        # Simple integration assuming constant velocity over dt
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi

        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = euler_to_quaternion(0, 0, self.theta)
        self.tf_broadcaster.sendTransform(t)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = euler_to_quaternion(0, 0, self.theta)

        # Set covariance (basic values)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # yaw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[35] = 0.01  # vth

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
