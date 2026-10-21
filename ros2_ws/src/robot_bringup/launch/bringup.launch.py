#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""小车一键启动（ROS 2 launch 版）

启动内容：
    1) 底盘   base_driver/chassis_node          → /cmd_vel ⇄ /odom + TF
    2) 雷达   ldlidar/ld14p.launch.py           → /scan + base_link→base_laser
    3) SLAM   slam_toolbox（建图 / 定位二选一）   → /map + map→odom
    4) 网页   robot_ui.app（05_robot_ui，六屏）  → 遥控/航点/巡逻/探索/编辑/功能
              ⚠ 由 start_robot.sh 在 launch 外拉起；本 launch 只在 phone:=true 时打一行
              提示（旧 phone_teleop.py 已归档到 01_Project/bak_phone_teleop_20261020/）
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
                            GroupAction, IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
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
    ekf = LaunchConfiguration('ekf')
    bag = LaunchConfiguration('bag')
    bag_dir = LaunchConfiguration('bag_dir')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

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
        DeclareLaunchArgument('nav2_params_file', default_value=os.path.join(
            get_package_share_directory('robot_bringup'), 'params', 'nav2_params.yaml'),
            description='Nav2 参数文件；与 slam_toolbox 的 params_file 分开'),
        DeclareLaunchArgument('ekf', default_value='false',
                              description='启动 robot_localization EKF（卡-02：轮速+IMU 融合）'
                                          '；开启后 odom→base_link 改由 EKF 发布'),
        DeclareLaunchArgument('bag', default_value='false', description='是否同时录 rosbag'),
        DeclareLaunchArgument('bag_dir', default_value=DEFAULT_BAG_DIR,
                              description='rosbag 输出目录'),
    ]

    # 1) 底盘
    #   ⚠ 卡-02 禁止行为：odom→base_link 只能由**一个**节点发布。
    #     ekf:=false → 底盘发（现状，兼容旧流程）
    #     ekf:=true  → 底盘停发，改由 EKF 发（否则两个发布者互相打架）
    chassis = Node(package='base_driver', executable='chassis_node',
                   name='base_driver', output='screen',
                   parameters=[{'publish_tf': True}],
                   condition=UnlessCondition(ekf))
    chassis_no_tf = Node(package='base_driver', executable='chassis_node',
                         name='base_driver', output='screen',
                         parameters=[{'publish_tf': False}],
                         condition=IfCondition(ekf))

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

    # 4) 手机网页 —— 已改为 robot_ui.app（六屏：遥控/航点/巡逻/探索/编辑/功能），
    #    由 start_robot.sh 在外部拉起（`python3 -m robot_ui.app`，cwd=03_SLAM）。
    #    旧 phone_teleop.py（单屏、无探索/编辑）已于 2026-10-20 归档到
    #    01_Project/bak_phone_teleop_20261020/；本 launch 不再拉起它，
    #    phone:=true 只会提示一句，不会报"文件不存在"。
    phone_proc = LogInfo(
        msg=['⚠ phone:=true 已废弃：网页服务现在总是 robot_ui.app，'
             '由 start_robot.sh 启动（若没看到网页，检查 /tmp/robot_ui.log）'],
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
                          'params_file': nav2_params_file,
                          'cmd_vel_out': '/cmd_vel',
                          'map_mode': mode,
                          'watchdog': 'true'}.items(),   # P23：恢复启用（§9.3 欠账）。9/23 雷达
                          # USB 抖动事故证明需要它兜底：/odom 断流 1.5s → 零速 + estop 取消导航
        condition=IfCondition(nav2),
    )

    # 5c) 可选：EKF 融合（卡-02）—— /odom + /imu/data → /odometry/filtered
    #     与底盘联动：ekf:=true 时 L2 用 chassis_no_tf（底盘停发 TF），TF 交给 EKF
    ekf_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ekf.launch.py')),
        condition=IfCondition(ekf),
    )

    # 6) 可选：录包（复盘用）
    bag_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/scan', '/odom', '/tf', '/tf_static', '/map',
             '/goal_pose', '/cmd_vel_raw', '/cmd_vel_nav',
             '/cmd_vel_manual', '/cmd_vel_muxed', '/cmd_vel',
             '/control/command', '/emergency_stop', '/battery'],
        output='screen',
        condition=IfCondition(bag),
    )

    banner = LogInfo(msg=['一键启动（launch）：mode=', mode,
                          ' ｜ 导航监听 nav=', nav,
                          ' ｜ nav2 栈 nav2=', nav2,
                          ' ｜ EKF 融合 ekf=', ekf,
                          ' ｜ 手机网页端口 ', port,
                          ' ｜ Ctrl-C 停止全部'])

    # IncludeLaunchDescription 的参数会写入当前上下文；SLAM 与 Nav2 都声明
    # params_file，必须隔离作用域，避免 Nav2 继承建图 YAML 后按缺省 DWB 启动失败。
    def scoped(action):
        return GroupAction(actions=[action], scoped=True)

    return LaunchDescription(declared + [banner, chassis, chassis_no_tf, scoped(lidar),
                                         scoped(slam_mapping), scoped(slam_localization),
                                         phone_proc, nav_proc, scoped(nav2_stack), scoped(ekf_stack),
                                         bag_proc])
