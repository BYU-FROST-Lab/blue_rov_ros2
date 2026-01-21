#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
# from scipy.spatial.transform import Rotation as R

from dvl_msgs.msg import DVL, DVLDR


class DVLToTwist(Node):

    def __init__(self):
        super().__init__('dvl_to_twist')


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

        self.pub_odom = self.create_publisher(
          Odometry,
          'dvl/odometry',
          10
        )
        self.sub_odom = self.create_subscription(
            DVLDR,
            'dvl/position',
            self.position_callback,
            qos_profile_sensor_data
        )


        self.get_logger().info('DVL → TwistWithCovarianceStamped converter started')

    def position_callback(self, msg: DVLDR):
        """Convert DVL dead-reckoned position to nav_msgs/Odometry."""

        odom = Odometry()

        # ---------- Header ----------
        odom.header.frame_id = 'map'
        odom.child_frame_id = msg.header.frame_id

        # DVLDR time is float64 seconds
        sec = int(msg.time)
        nanosec = int((msg.time - sec) * 1e9)
        odom.header.stamp.sec = sec
        odom.header.stamp.nanosec = nanosec

        # ---------- Position ----------
        odom.pose.pose.position.x = msg.position.x
        odom.pose.pose.position.y = msg.position.y
        odom.pose.pose.position.z = msg.position.z

        # ---------- Orientation (Euler → quaternion via scipy) ----------
        # quat = R.from_euler('xyz', [msg.roll, msg.pitch, msg.yaw]).as_quat()
        # odom.pose.pose.orientation.x = quat[0]
        # odom.pose.pose.orientation.y = quat[1]
        # odom.pose.pose.orientation.z = quat[2]
        # odom.pose.pose.orientation.w = quat[3]

        # ---------- Pose covariance ----------
        # Position variance from pos_std (assumed isotropic)
        var = msg.pos_std ** 2
        odom.pose.covariance[0] = var
        odom.pose.covariance[7] = var
        odom.pose.covariance[14] = var

        self.pub_odom.publish(odom)

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
