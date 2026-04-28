#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class SbgOdomTf(Node):
    """
    Republishes imu/odometry as a TF transform.

    The SBG driver publishes imu/odometry with frame_id="odom" and
    child_frame_id="bluerov2/base_link". This node converts that odometry
    message into a TF broadcast so the full frame tree is available:
        odom -> bluerov2/base_link
    """

    def __init__(self):
        super().__init__('sbg_odom_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'imu/odometry', self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry):
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SbgOdomTf())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
