import copy
import threading
import time

import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf2_ros
import ros2_numpy


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")

        self.global_map = None
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        self.initialized = False

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("localization_threshold", 0.8),
                ("fov", 6.28319),
                ("fov_far", 300),
            ],
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)

        self.create_subscription(PointCloud2, "/cloud_registered", self.cb_save_cur_scan, 10)
        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.cb_initialize_pose, 10)

        threading.Thread(target=self.thread_localization, daemon=True).start()

    def msg_to_array(self, pc_msg):
        pc_array = ros2_numpy.numpify(pc_msg)
        pc = np.zeros([len(pc_array), 3])
        pc[:, 0] = pc_array["x"]
        pc[:, 1] = pc_array["y"]
        pc[:, 2] = pc_array["z"]
        return pc

    def publish_point_cloud(self, publisher, header, pc):
        data = np.zeros(
            len(pc),
            dtype=[
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
            ],
        )
        data["x"] = pc[:, 0]
        data["y"] = pc[:, 1]
        data["z"] = pc[:, 2]
        msg = ros2_numpy.msgify(PointCloud2, data)
        msg.header = header
        publisher.publish(msg)

    def cb_save_cur_scan(self, msg):
        pc = self.msg_to_array(msg)
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc)

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg

    def cb_initialize_pose(self, msg):
        initial_pose = self.pose_to_mat(msg.pose.pose)
        if self.cur_scan is not None:
            self.global_localization(initial_pose)

    def pose_to_mat(self, pose):
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = tf2_ros.transformations.quaternion_matrix(quat)[:3, :3]
        return trans

    def global_localization(self, pose_estimation):
        scan_tobe_mapped = copy.copy(self.cur_scan)
        transformation, fitness = self.registration_at_scale(scan_tobe_mapped, self.global_map, pose_estimation)
        if fitness > self.get_parameter("localization_threshold").value:
            self.T_map_to_odom = transformation
            self.publish_tf(transformation)

    def registration_at_scale(self, scan, map, initial):
        result = o3d.pipelines.registration.registration_icp(
            scan,
            map,
            1.0,
            initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        return result.transformation, result.fitness

    def publish_tf(self, transform):
        odom_msg = Odometry()
        xyz = transform[:3, 3]
        quat = tf2_ros.transformations.quaternion_from_matrix(transform)
        odom_msg.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        self.pub_map_to_odom.publish(odom_msg)

    def thread_localization(self):
        while rclpy.ok():
            if self.cur_scan is not None:
                self.global_localization(self.T_map_to_odom)
            time.sleep(1.0 / self.get_parameter("freq_localization").value)


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
