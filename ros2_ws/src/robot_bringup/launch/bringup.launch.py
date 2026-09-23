#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""小车一键启动（ROS 2 launch 版）

启动内容：
    1) 底盘   base_driver/chassis_node          → /cmd_vel ⇄ /odom + TF
    2) 雷达   ldlidar/ld14p.launch.py           → /scan + base_link→base_laser
    3) SLAM   slam_toolbox（建图 / 定位二选一）   → /map + map→odom
    4) 手机   01_Project/03_SLAM/phone_teleop.py → 网页遥控 + 实时地图 + 保存地图 + 点选目标
    5) 可选   goal_nav.py --listen              → 连续导航：手机点一个点就去一个点（nav:=true）
    6) 可选   录 rosbag（/scan /odom /tf /cmd_vel /map）

用法：
    ros2 launch robot_bringup bringup.launch.py                     # 建图模式（默认）
    ros2 launch robot_bringup bringup.launch.py mode:=localization  # 定位模式（导航用）
    ros2 launch robot_bringup bringup.launch.py mode:=localization nav:=true   # 定位 + 导航监听
    ros2 launch robot_bringup bringup.launch.py phone:=false bag:=true

参数：
    mode      mapping | localization   （默认 mapping）
    phone     是否启动手机遥控网页      （默认 true）
    port      手机网页端口              （默认 8080）
    view      手机端地图视野（米）       （默认 8.0）
    nav       是否同时起导航监听（goal_nav --listen，连续模式）（默认 false）
              ⚠ 开了之后，手机地图上点一下车就会开过去（到点后继续等下一个目标）
    bag       是否同时录 rosbag         （默认 false）
    bag_dir   录包输出目录              （默认 ~/RobotCode/logs/bags/run_<月日_时分>）

停止：**Ctrl-C 即可** —— launch 会把所有子进程一并收掉（底盘节点会发 AT+MT_STOP，
      手机遥控会发零速度）；比逐个 pkill 干净得多。
"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

SLAM_TOOLS = os.path.expanduser('~/RobotCode/01_Project/03_SLAM')
DEFAULT_BAG_DIR = os.path.join(os.path.expanduser('~/RobotCode/logs/bags'),
                               'run_' + datetime.now().strftime('%m%d_%H%M%S'))


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    phone = LaunchConfiguration('phone')
    port = LaunchConfiguration('port')
    view = LaunchConfiguration('view')
    nav = LaunchConfiguration('nav')
    nav2 = LaunchConfiguration('nav2')
    bag = LaunchConfiguration('bag')
    bag_dir = LaunchConfiguration('bag_dir')

    slam_share = get_package_share_directory('slam_toolbox')
    lidar_share = get_package_share_directory('ldlidar')

    declared = [
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='mapping=建图 ｜ localization=定位（导航）'),
        DeclareLaunchArgument('phone', default_value='true',
                              description='是否启动手机遥控网页'),
        DeclareLaunchArgument('port', default_value='8080', description='手机网页端口'),
        DeclareLaunchArgument('view', default_value='8.0', description='手机端地图视野（米）'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='是否同时起导航监听（goal_nav --listen：手机点一个点就去一个点）'),
        # nav2:=true 时启动 nav2 全栈（bt_navigator + DWB + collision_monitor）。
        # ⚠ 与 nav:=true（自研 goal_nav --listen）互斥：两者都直接发 /cmd_vel，同开会打架。
        #   手机网页点目标 → /goal_pose → nav2_goal_bridge（净化）→ navigate_to_pose。
        DeclareLaunchArgument('nav2', default_value='false',
                              description='启动 nav2 全栈（默认 false；开启时自动忽略 nav）'),
        DeclareLaunchArgument('bag', default_value='false', description='是否同时录 rosbag'),
        DeclareLaunchArgument('bag_dir', default_value=DEFAULT_BAG_DIR,
                              description='rosbag 输出目录'),
    ]

    # 1) 底盘
    chassis = Node(package='base_driver', executable='chassis_node',
                   name='base_driver', output='screen')

    # 2) 雷达（含 base_link→base_laser 静态 TF）
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_share, 'launch', 'ld14p.launch.py')))

    # 3) SLAM：按 mode 二选一
    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
    )
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'localization_launch.py')),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
    )

    # 4) 手机遥控网页（含「保存地图」按钮与地图点选目标）
    phone_proc = ExecuteProcess(
        cmd=['python3', os.path.join(SLAM_TOOLS, 'phone_teleop.py'),
             '-p', port, '-r', view],
        output='screen',
        condition=IfCondition(phone),
    )

    # 5) 可选：导航监听（连续模式：到达一个点后继续等下一个目标）
    #    nav2:=true 时自动屏蔽 —— goal_nav 与 nav2 都直接发 /cmd_vel，不能同开
    nav_proc = ExecuteProcess(
        cmd=['python3', os.path.join(SLAM_TOOLS, 'goal_nav.py'), '--listen'],
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", nav, "' == 'true' and '", nav2, "' == 'false'"])),
    )

    # 5b) 可选：nav2 全栈（collision_monitor 在 nav2.launch.py 内一并起）
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nav2.launch.py')),
        launch_arguments={'goal_topic': '/goal_pose',
                          'cmd_vel_out': '/cmd_vel',
                          'watchdog': 'true'}.items(),   # P23：恢复启用（§9.3 欠账）。9/23 雷达
                          # USB 抖动事故证明需要它兜底：/odom 断流 1.5s → 零速 + estop 取消导航
        condition=IfCondition(nav2),
    )

    # 6) 可选：录包（复盘用）
    bag_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/scan', '/odom', '/tf', '/tf_static', '/cmd_vel', '/map'],
        output='screen',
        condition=IfCondition(bag),
    )

    banner = LogInfo(msg=['一键启动（launch）：mode=', mode,
                          ' ｜ 导航监听 nav=', nav,
                          ' ｜ nav2 栈 nav2=', nav2,
                          ' ｜ 手机网页端口 ', port,
                          ' ｜ Ctrl-C 停止全部'])

    return LaunchDescription(declared + [banner, chassis, lidar,
                                         slam_mapping, slam_localization,
                                         phone_proc, nav_proc, nav2_stack, bag_proc])
