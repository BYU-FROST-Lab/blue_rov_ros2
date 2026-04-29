#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


def _pose_to_matrix(pose) -> np.ndarray:
    p = pose.position
    q = pose.orientation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _tf_to_matrix(tf_stamped: TransformStamped) -> np.ndarray:
    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def _matrix_to_tf(M: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x = float(M[0, 3])
    tf.transform.translation.y = float(M[1, 3])
    tf.transform.translation.z = float(M[2, 3])
    q = Rotation.from_matrix(M[:3, :3]).as_quat()
    tf.transform.rotation.x = float(q[0])
    tf.transform.rotation.y = float(q[1])
    tf.transform.rotation.z = float(q[2])
    tf.transform.rotation.w = float(q[3])
    return tf


class SbgOdomTf(Node):
    """
    Converts imu/odometry into an odom -> bluerov2/base_link TF.

    The SBG driver publishes imu/odometry with:
      header.frame_id  = "odom"
      child_frame_id   = "bluerov2/imu_link"

    This node looks up the static imu_link -> base_link offset from the URDF
    (published by robot_state_publisher) and composes:

        T_odom_base = T_odom_imu * T_imu_base

    so that base_link stays the URDF root while imu_link is correctly
    described as a child with a measured physical offset.
    """

    BASE_FRAME = 'bluerov2/base_link'

    def __init__(self):
        super().__init__('sbg_odom_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, 'imu/odometry', self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry):
        imu_frame = msg.child_frame_id  # "bluerov2/imu_link"
        odom_frame = msg.header.frame_id  # "odom"

        # Pose of imu_link in odom frame (from SBG odometry)
        M_odom_imu = _pose_to_matrix(msg.pose.pose)

        # Static offset: pose of base_link in imu_link frame
        # lookup_transform(target, source) converts source -> target coordinates
        # so lookup("bluerov2/imu_link", "bluerov2/base_link") gives T_imu_base
        try:
            tf_imu_base = self.tf_buffer.lookup_transform(
                imu_frame,
                self.BASE_FRAME,
                rclpy.time.Time(),
            )
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup {imu_frame} -> {self.BASE_FRAME} failed: {e}',
                throttle_duration_sec=5.0,
            )
            return

        M_imu_base = _tf_to_matrix(tf_imu_base)

        # Compose: T_odom_base = T_odom_imu * T_imu_base
        M_odom_base = M_odom_imu @ M_imu_base

        out = _matrix_to_tf(M_odom_base, odom_frame, self.BASE_FRAME, msg.header.stamp)
        self.tf_broadcaster.sendTransform(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SbgOdomTf())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
