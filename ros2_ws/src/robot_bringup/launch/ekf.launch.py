#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EKF 融合启动（轮速里程计 + IMU）—— 卡-02

启动内容：
    1) imu_driver.py              电源板 IMU（ICM-42670，I2C-6@0x69）→ /imu/data
    2) static_transform_publisher base_link → imu_link（IMU 安装位姿）
    3) ekf_filter_node            robot_localization → /odometry/filtered + TF odom→base_link

⚠ 卡-02 禁止行为：odom→base_link 只能由一个节点发布。
   本文件把 TF 交给 EKF（ekf.yaml 的 publish_tf: true），所以启用时必须让 base_driver
   停发 TF —— bringup.launch.py 在 ekf:=true 时会自动把 chassis 的 publish_tf 置 false。

⚠ 首次使用必须先标定 IMU 安装朝向（清单步骤 2.2）：
   实测本电源板 IMU 的 **+z 轴朝下**（静止时 accel_z ≈ -9.81 m/s²，即 -1g）。
   若 base_link→imu_link 用默认 (0,0,0)，EKF 收到的偏航角速度**符号是反的**，
   会与轮速里程计互相打架（表现为航向越走越歪）。

   5 秒判定法：启动后手推小车**向左（逆时针）**转，
       ros2 topic echo /imu/data --field angular_velocity.z
   · 读数为**正** → IMU 的 z 与 base_link 同向 → imu_rpy 保持 "0 0 0"
   · 读数为**负** → z 轴反了            → imu_rpy 改成 "3.14159 0 0"

参数：
    imu            是否启动 imu_driver（默认 true；单独调 EKF 时可 false）
    ekf_params     EKF 参数文件（默认本包 config/ekf.yaml）
    imu_rate       /imu/data 发布频率（默认 100，实测可达 ~450 Hz）
    imu_x/y/z      IMU 相对 base_link 的位置，米（默认 0；建议按实测填）
    imu_roll/pitch/yaw  IMU 相对 base_link 的姿态，弧度（默认 0；见上方朝向标定）
    gyro_bias      陀螺零偏 x y z（rad/s），默认 2026-10-15 实测值

用法：
    ros2 launch robot_bringup ekf.launch.py                  # 独立调试
    ros2 launch robot_bringup bringup.launch.py ekf:=true     # 与底盘/SLAM 一起（推荐）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

SLAM_TOOLS = os.path.expanduser('~/RobotCode/01_Project/03_SLAM')

# 2026-10-15 实测零偏（静止 60 s / 3744 点，见清单 §3）
# 逗号分隔的单个 token —— ExecuteProcess 会把含空格的字符串当成一个参数，argparse 解不开
DEFAULT_GYRO_BIAS = '-0.008333,-0.007322,0.005007'

# IMU 安装位姿
#   姿态：2026-10-15 实测定版。手推车**逆时针**转 180°（真值 +180°），
#         imu_driver.py --yaw-test 积分得 **-177.4°** → IMU 的 +z 轴朝下、与 base_link 反了
#         → roll = 3.14159 把 z 翻回来。
#         （|177.4| vs 180 差 1.4%，同时验证陀螺 131.072 LSB/dps 系数正确）
#   位置：【仍待实测】电源板 IMU 相对车体中心，单位米。
#         注意：当前配置只融合 vyaw，而**角速度与安装位置无关**（刚体上各点相同），
#         所以 xyz 目前不影响 EKF 结果；但换配置（融合加速度）前必须补测。
DEFAULT_IMU_XYZ = ('0.0', '0.0', '0.0')
DEFAULT_IMU_RPY = ('3.14159', '0.0', '0.0')


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')

    imu = LaunchConfiguration('imu')
    ekf_params = LaunchConfiguration('ekf_params')
    imu_rate = LaunchConfiguration('imu_rate')
    imu_x = LaunchConfiguration('imu_x')
    imu_y = LaunchConfiguration('imu_y')
    imu_z = LaunchConfiguration('imu_z')
    imu_roll = LaunchConfiguration('imu_roll')
    imu_pitch = LaunchConfiguration('imu_pitch')
    imu_yaw = LaunchConfiguration('imu_yaw')
    gyro_bias = LaunchConfiguration('gyro_bias')

    declared = [
        DeclareLaunchArgument('imu', default_value='true',
                              description='是否启动 imu_driver（/imu/data）'),
        DeclareLaunchArgument('ekf_params',
                              default_value=os.path.join(bringup_share, 'config',
                                                         'ekf.yaml'),
                              description='EKF 参数文件'),
        DeclareLaunchArgument('imu_rate', default_value='100.0',
                              description='/imu/data 发布频率 Hz'),
        DeclareLaunchArgument('imu_x', default_value=DEFAULT_IMU_XYZ[0],
                              description='IMU 位置 x（米）'),
        DeclareLaunchArgument('imu_y', default_value=DEFAULT_IMU_XYZ[1],
                              description='IMU 位置 y（米）'),
        DeclareLaunchArgument('imu_z', default_value=DEFAULT_IMU_XYZ[2],
                              description='IMU 位置 z（米）'),
        DeclareLaunchArgument('imu_roll', default_value=DEFAULT_IMU_RPY[0],
                              description='IMU 姿态 roll（弧度）；z 轴朝下时填 3.14159'),
        DeclareLaunchArgument('imu_pitch', default_value=DEFAULT_IMU_RPY[1],
                              description='IMU 姿态 pitch（弧度）'),
        DeclareLaunchArgument('imu_yaw', default_value=DEFAULT_IMU_RPY[2],
                              description='IMU 姿态 yaw（弧度）'),
        DeclareLaunchArgument('gyro_bias', default_value=DEFAULT_GYRO_BIAS,
                              description='陀螺零偏 x y z（rad/s），2026-10-15 实测'),
    ]

    # 1) IMU 驱动（脚本在 01_Project 下，用 ExecuteProcess 起，与 phone_teleop 同法）
    imu_proc = ExecuteProcess(
        cmd=['python3', os.path.join(SLAM_TOOLS, 'imu_driver.py'),
             '--rate', imu_rate,
             '--frame', 'imu_link',
             # 必须用 "--gyro-bias=<值>" 形式：值以 '-' 开头且含逗号时，
             # 拆成 "--gyro-bias" "<值>" 会被 argparse 当成选项，报 "expected one argument"
             PythonExpression(["'--gyro-bias=", gyro_bias, "'"])],
        output='screen',
        condition=IfCondition(imu),
    )

    # 2) IMU 安装位姿静态 TF（EKF 靠它把 IMU 数据变换到 base_link）
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['--x', imu_x, '--y', imu_y, '--z', imu_z,
                   '--roll', imu_roll, '--pitch', imu_pitch, '--yaw', imu_yaw,
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
    )

    # 3) EKF 本体
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ParameterFile(ekf_params, allow_substs=True)],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    banner = LogInfo(msg=['EKF 启动：IMU 位姿 xyz=(', imu_x, ', ', imu_y, ', ', imu_z,
                          ') rpy=(', imu_roll, ', ', imu_pitch, ', ', imu_yaw, ')',
                          ' ｜ TF odom→base_link 由 EKF 发布（底盘已停发）',
                          ' ｜ 未做朝向标定前，偏航角速度符号可能是反的（清单 2.2）'])

    return LaunchDescription(declared + [banner, imu_proc, imu_tf, ekf_node])
