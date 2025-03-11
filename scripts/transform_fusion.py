import copy
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import tf_transformations
import tf2_ros


class TransformFusion(Node):
    def __init__(self):
        super().__init__("transform_fusion")

        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_localization = self.create_publisher(Odometry, "/localization", 10)

        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 10)
        self.create_subscription(Odometry, "/map_to_odom", self.cb_save_map_to_odom, 10)

        self.freq_pub_localization = 50
        threading.Thread(target=self.transform_fusion, daemon=True).start()

    def pose_to_mat(self, pose_msg):
        trans = np.eye(4)
        trans[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans

    def transform_fusion(self):
        rate = self.create_rate(self.freq_pub_localization)
        while rclpy.ok():
            if self.cur_odom_to_baselink is None:
                continue

            cur_odom = copy.copy(self.cur_odom_to_baselink)
            if self.cur_map_to_odom is not None:
                T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
            else:
                T_map_to_odom = np.eye(4)

            T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)

            xyz = tf_transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf_transformations.quaternion_from_matrix(T_map_to_base_link)

            localization = Odometry()
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            localization.twist = cur_odom.twist

            localization.header.stamp = self.get_clock().now().to_msg()
            localization.header.frame_id = "map"
            localization.child_frame_id = "body"
            self.pub_localization.publish(localization)

            time.sleep(rate)

    def cb_save_cur_odom(self, msg):
        self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg):
        self.cur_map_to_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
