"""Evaluate full launch scopes while replacing every process action with a recorder.

No ROS process, serial device, or motor topic is started.
"""
import os
import sys
from unittest.mock import patch

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from pathlib import Path

launch_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    get_package_share_directory('robot_bringup'), 'launch', 'bringup.launch.py')

for mode in ('mapping', 'localization'):
    captured = {}

    def record_node(action, context):
        executable = perform_substitutions(context, normalize_to_list_of_substitutions(action.node_executable))
        if executable in ('controller_server', 'async_slam_toolbox_node', 'localization_slam_toolbox_node'):
            evaluated = evaluate_parameters(context, action._Node__parameters)
            paths = [str(item) for item in evaluated if isinstance(item, Path)]
            if executable == 'controller_server':
                captured[executable] = context.launch_configurations.get('params_file')
                with open(paths[0]) as source:
                    actual = yaml.safe_load(source)
                assert 'MPPI' in actual['controller_server']['ros__parameters']['FollowPath']['plugin']
            else:
                captured[executable] = paths[0]
        return []

    service = LaunchService(argv=[])
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file),
                                 launch_arguments={'mode': mode, 'nav2': 'true',
                                                   'ekf': 'true', 'phone': 'false'}.items())]))
    with patch.object(Node, 'execute', record_node), patch.object(ExecuteProcess, 'execute', lambda *_: []):
        assert service.run() == 0
    controller_file = captured['controller_server']
    assert controller_file.endswith('/params/nav2_params.yaml'), captured
    with open(controller_file) as source:
        params = yaml.safe_load(source)
    assert 'MPPI' in params['controller_server']['ros__parameters']['FollowPath']['plugin']
    slam_node = 'async_slam_toolbox_node' if mode == 'mapping' else 'localization_slam_toolbox_node'
    expected = 'mapper_params_online_async.yaml' if mode == 'mapping' else 'mapper_params_localization.yaml'
    assert captured[slam_node].endswith(expected), captured
    print(mode, 'SLAM:', captured[slam_node], 'NAV2:', controller_file, 'PASS')
