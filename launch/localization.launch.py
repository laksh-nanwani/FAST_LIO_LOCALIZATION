from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare arguments
            DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz"),
            DeclareLaunchArgument("map", default_value="", description="Path to PCD map file"),
            DeclareLaunchArgument("pcd_map_topic", default_value="/map", description="Topic to publish PCD map"),
            # Load parameters from yaml file
            Node(
                package="fast_lio_localization",
                executable="fastlio_mapping",
                name="laserMapping",
                output="screen",
                parameters=[os.path.join(os.getenv("FAST_LIO_LOCALIZATION_DIR"), "config", "mid360.yaml")],
            ),
            # Global localization node
            Node(
                package="fast_lio_localization",
                executable="global_localization.py",
                name="global_localization",
                output="screen",
            ),
            # Transform fusion node
            Node(
                package="fast_lio_localization",
                executable="transform_fusion.py",
                name="transform_fusion",
                output="screen",
            ),
            # PCD to PointCloud2 publisher
            Node(
                package="pcl_ros",
                executable="pcd_to_pointcloud",
                name="map_publisher",
                output="screen",
                arguments=[
                    LaunchConfiguration("map"),
                    "5",
                    "_frame_id:=map",
                    "cloud_pcd:=" + LaunchConfiguration("pcd_map_topic"),
                ],
            ),
            # Conditional RViz launch
            GroupAction(
                [
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz",
                        arguments=[
                            "-d",
                            os.path.join(os.getenv("FAST_LIO_LOCALIZATION_DIR"), "rviz_cfg", "localization.rviz"),
                        ],
                    )
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
