import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_params_file = os.path.join(get_package_share_directory("slam_toolbox"),
                                       'config', 'mapper_params_online_async.yaml')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # —— 本机定制（2026-09-20，RK3588/Toybrick 版 ROS2 环境适配）——
    # 1) 原版用 nav2_common.launch.HasNodeParams 做"参数文件回退"检查，
    #    本机 ROS2 打包无 nav2_common，故简化为直接使用 params_file
    #    （默认值即本包自带配置，不存在回退需求）。
    # 2) use_sim_time 默认由 true 改为 false（真机，无仿真时钟）。
    start_async_slam_toolbox_node = Node(
        parameters=[
          params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
