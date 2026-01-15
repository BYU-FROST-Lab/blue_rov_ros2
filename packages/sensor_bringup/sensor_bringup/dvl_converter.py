#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import TwistWithCovarianceStamped
from builtin_interfaces.msg import Time

from dvl_msgs.msg import DVL


class DVLToTwist(Node):

    def __init__(self):
        super().__init__('dvl_to_twist')

        # TODO if we want control of frame id 
        self.declare_parameter('frame_id', 'dvl_link')
        self.frame_id = self.get_parameter('frame_id').value


        self.sub = self.create_subscription(
            DVL,
            'dvl/data',
            self.dvl_callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            TwistWithCovarianceStamped,
            'dvl/twist',
            10
        )

        self.get_logger().info('DVL → TwistWithCovarianceStamped converter started')

    def dvl_callback(self, msg: DVL):

        if not msg.velocity_valid:
            self.get_logger().warn('Received invalid DVL velocity')
            return

        twist_msg = TwistWithCovarianceStamped()

        # ---------- Header ----------
        twist_msg.header.frame_id = msg.header.frame_id
        twist_msg.header.stamp = self._time_from_microseconds(
            msg.time_of_validity
        )

        # ---------- Linear velocity -------------
        twist_msg.twist.twist.linear.x = msg.velocity.x
        twist_msg.twist.twist.linear.y = msg.velocity.y
        twist_msg.twist.twist.linear.z = msg.velocity.z

        # ---------- Covariance -------------------
        # Assumes msg.covariance is 3x3 (row-major) for linear velocity
        # Populate the 6x6 Twist covariance matrix
        twist_msg.twist.covariance[0:3] = msg.covariance[0:3]
        twist_msg.twist.covariance[6:9] = msg.covariance[3:6]
        twist_msg.twist.covariance[12:15] = msg.covariance[6:9]


        self.pub.publish(twist_msg)

    @staticmethod
    def _time_from_microseconds(us: int) -> Time:
        t = Time()
        t.sec = us // 1_000_000
        t.nanosec = (us % 1_000_000) * 1_000
        return t


def main(args=None):
    rclpy.init(args=args)
    node = DVLToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
