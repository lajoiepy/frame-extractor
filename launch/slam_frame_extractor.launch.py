import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    loop_detection_node = Node(package='slam_frame_extractor',
                               executable='loop_closure_detection_node.py',
                               name='slam_frame_extractor_loop_closure_detection',
                               parameters=[
                                   LaunchConfiguration('config')
                               ],
                               namespace=LaunchConfiguration('namespace'))

    map_manager_node = Node(package='slam_frame_extractor',
                            executable='map_manager',
                            name='slam_frame_extractor_map_manager',
                            parameters=[
                                LaunchConfiguration('config')
                            ],
                            prefix=LaunchConfiguration('launch_prefix'),
                            namespace=LaunchConfiguration('namespace'))

    pose_graph_manager_node = Node(package='slam_frame_extractor',
                                   executable='pose_graph_manager',
                                   name='slam_frame_extractor_pose_graph_manager',
                                   parameters=[
                                       LaunchConfiguration('config')
                                   ],
                                   prefix=LaunchConfiguration('launch_prefix'),
                                   namespace=LaunchConfiguration('namespace'))

    return [
        loop_detection_node,
        map_manager_node,
        pose_graph_manager_node,
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('config_path',
                              default_value=os.path.join(
                                  get_package_share_directory('slam_frame_extractor'),
                                  'config', 'slam_frame_extractor/'),
                              description=''),
        DeclareLaunchArgument('config_file',
                              default_value='default_kitti.yaml',
                              description=''),
        DeclareLaunchArgument('config',
                              default_value=[
                                  LaunchConfiguration('config_path'),
                                  LaunchConfiguration('config_file')
                              ],
                              description=''),
        DeclareLaunchArgument(
            'launch_prefix',
            default_value='',
            description=
            'For debugging purpose, it fills prefix tag of the nodes, e.g., "xterm -e gdb -ex run --args"'
        ),
        DeclareLaunchArgument('log_level',
                              default_value='error',
                              description=''),
        OpaqueFunction(function=launch_setup),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_frame_extractor'), 'launch',
                             'odometry', 'rtabmap_odometry.launch.py')),
            launch_arguments={
                'log_level': LaunchConfiguration('log_level'),
            }.items(),
        )
    ])
