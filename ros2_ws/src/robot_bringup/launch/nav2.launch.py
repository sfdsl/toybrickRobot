#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav2 导航栈启动（本机定制版）

为什么不直接 include `nav2_bringup/navigation_launch.py`：
    官方 launch 里 controller / velocity_smoother 的 cmd_vel 重映射是**硬编码**的
    （controller → cmd_vel_nav → smoother → cmd_vel），没法在中间插 collision_monitor。
    所以这里照抄官方的节点定义，只重排这条链路：

      controller_server → cmd_vel_raw → velocity_smoother → cmd_vel_nav
                                                          → collision_monitor → cmd_vel（底盘）
                                                          （硬安全层：前方减速/急停）

**前置**：定位（slam_toolbox localization）必须已经在跑（`robotnav`）——
本 launch 只管导航，不启 amcl / map_server（避免 /map 两个发布者打架）。

用法：
    ros2 launch robot_bringup nav2.launch.py                       # 正常导航
    ros2 launch robot_bringup nav2.launch.py collision:=false      # 不挂硬安全层（调试用）
    ros2 launch robot_bringup nav2.launch.py bridge:=false         # 不起 /goal_pose 桥接
    ros2 launch robot_bringup nav2.launch.py watchdog:=false       # 不起 /odom 看门狗
    ros2 launch robot_bringup nav2.launch.py cmd_vel_out:=/test_cmd_vel   # P2：不动车验链路
    ros2 launch robot_bringup nav2.launch.py goal_topic:=/test_goal_pose  # P2：目标也走测试话题

参数：
    params_file   nav2 参数文件（默认本包 params/nav2_params.yaml）
    collision     是否起 collision_monitor（默认 true）
    bridge        是否起 /goal_pose 桥接（默认 true）
    watchdog      是否起 /odom 断流看门狗（默认 true）
    cmd_vel_out   最终速度话题（默认 cmd_vel）
    goal_topic    桥接订阅的目标话题（默认 /goal_pose）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            LogInfo, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

SLAM_TOOLS = os.path.expanduser('~/RobotCode/01_Project/03_SLAM')

# nav2 生命周期节点（不含 collision_monitor —— 它是普通节点）
LIFECYCLE_NODES = ['controller_server',
                   'smoother_server',
                   'planner_server',
                   'behavior_server',
                   'bt_navigator',
                   'waypoint_follower',
                   'velocity_smoother']


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    collision = LaunchConfiguration('collision')
    bridge = LaunchConfiguration('bridge')
    watchdog = LaunchConfiguration('watchdog')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')
    goal_topic = LaunchConfiguration('goal_topic')

    # tf 用相对名，便于加命名空间
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={'use_sim_time': use_sim_time,
                            'autostart': autostart},
            convert_types=True),
        allow_substs=True)

    monitor_params = ParameterFile(
        RewrittenYaml(
            source_file=os.path.join(bringup_share, 'config', 'collision_monitor.yaml'),
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True),
        allow_substs=True)

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='真机恒为 false'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='自动拉起 nav2 生命周期节点'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(bringup_share, 'params',
                                                         'nav2_params.yaml'),
                              description='nav2 参数文件'),
        DeclareLaunchArgument('collision', default_value='true',
                              description='是否起 collision_monitor（硬安全层）'),
        DeclareLaunchArgument('bridge', default_value='true',
                              description='是否起 /goal_pose → nav2 桥接'),
        DeclareLaunchArgument('watchdog', default_value='true',
                              description='是否起 /odom 断流看门狗'),
        DeclareLaunchArgument('cmd_vel_out', default_value='cmd_vel',
                              description='最终速度话题（P2 不动车测试改 /test_cmd_vel）'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose',
                              description='桥接订阅的目标话题'),
    ]

    nodes = [
        # 1) 控制器：输出改名为 cmd_vel_raw，交给速度平滑
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[configured_params],
             remappings=remappings + [('cmd_vel', 'cmd_vel_raw')]),
        # 2) 路径平滑（对应自研 Chaikin 拐角平滑）
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen',
             parameters=[configured_params],
             remappings=remappings),
        # 3) 全局规划（SmacPlanner2D）
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[configured_params],
             remappings=remappings),
        # 4) 恢复行为（挡路 → 等待/自旋/后退）
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[configured_params],
             remappings=remappings),
        # 5) 行为树导航器（对外是 navigate_to_pose action）
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[configured_params],
             remappings=remappings),
        # 6) 多航点巡检（自研没有的能力）
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen',
             parameters=[configured_params],
             remappings=remappings),
        # 7) 速度平滑：cmd_vel_raw → cmd_vel_nav
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[configured_params],
             remappings=remappings + [('cmd_vel', 'cmd_vel_raw'),
                                      ('cmd_vel_smoothed', 'cmd_vel_nav')]),
        # 8) 生命周期管理
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time},
                         {'autostart': autostart},
                         {'node_names': LIFECYCLE_NODES}]),
    ]

    # 9) 硬安全层：cmd_vel_nav → cmd_vel（前方减速/急停）
    collision_node = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[monitor_params, {'cmd_vel_out_topic': cmd_vel_out}],
        condition=IfCondition(collision),
    )

    # 9b) ⚠ P2 实测：collision_monitor 是 **lifecycle 节点**，而 nav2 的
    #     `lifecycle_manager_navigation` **不管它**（官方 navigation_launch.py 也一样），
    #     所以不加下面这个 manager 的话它会一直停在 inactive：
    #       · 不订阅 cmd_vel_nav（实测 /cmd_vel_nav 的 Subscription count = 0）
    #       · 不发布 cmd_vel_out → **整条链路到它这里就断了**，硬安全层形同虚设
    #     症状很隐蔽：controller 照常 10 Hz 出速度、规划一切正常，只是车永远不动。
    safety_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_safety', output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['collision_monitor']}],
        condition=IfCondition(collision),
    )

    # 10) /goal_pose → nav2 action（手机页面零改动）
    bridge_proc = ExecuteProcess(
        cmd=['python3', os.path.join(SLAM_TOOLS, 'nav2_goal_bridge.py'),
             '--goal-topic', goal_topic, '--cmd-topic', cmd_vel_out],
        output='screen',
        condition=IfCondition(bridge),
    )

    # 11) /odom 断流看门狗（nav2 不管这条，自研安全层的底线）
    watchdog_proc = ExecuteProcess(
        cmd=['python3', os.path.join(SLAM_TOOLS, 'safety_watchdog.py')],
        output='screen',
        condition=IfCondition(watchdog),
    )

    banner = LogInfo(msg=['nav2 启动：参数=', params_file,
                          ' ｜ 硬安全层=', collision,
                          ' ｜ 桥接=', bridge,
                          ' ｜ 看门狗=', watchdog,
                          ' ｜ 速度输出=', cmd_vel_out,
                          ' ｜ 前置：定位(slam_toolbox)需已在跑'])

    return LaunchDescription(
        # ⚠ 不要用缓冲日志：P2 排查时踩过——RCUTILS_LOGGING_BUFFERED_STREAM=1 会把
        #   子进程的 rcutils 日志攒在缓冲区里，桥接的「收到目标点」等 INFO 迟迟不落地，
        #   看起来就像"目标没送达/回调没触发"，实际是日志没刷出来。真机排查要实时日志。
        [SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0')]
        + declared
        + [banner]
        + nodes
        + [collision_node, safety_manager, bridge_proc, watchdog_proc])
