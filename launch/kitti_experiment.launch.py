import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, PushLaunchConfigurations, PopLaunchConfigurations
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription


def launch_setup(context, *args, **kwargs):
    config_path = os.path.join(get_package_share_directory("slam_frame_extractor"),
                               "config", 'slam_frame_extractor/')
    config_file = "default_kitti.yaml"

    slam_processes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_frame_extractor"), "launch",
                         "slam_frame_extractor.launch.py")),
        launch_arguments={
            "namespace": "/slam_frame_extractor",
            "config_path": config_path,
            "config_file": config_file,
        }.items(),
    )

    bag_file = os.path.join(get_package_share_directory("slam_frame_extractor"),
                              "data", "KITTI00")

    bag_process = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_frame_extractor"),
                "launch",
                "sensors",
                "bag_kitti.launch.py",
            )),
        launch_arguments={
            "namespace": "/slam_frame_extractor",
            "bag_file": bag_file,
            "rate": "0.1"
        }.items(),
    )

    tf_process = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 camera_link camera_gray_left".split(" "),
        parameters=[]
    )

    return [
        PushLaunchConfigurations(),
        TimerAction(
            period='30',
            actions=[bag_process]),
        PopLaunchConfigurations(),
        PushLaunchConfigurations(),
        slam_processes,
        PopLaunchConfigurations(),
        tf_process,
    ]


def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])