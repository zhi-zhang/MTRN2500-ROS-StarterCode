#!/usr/bin/env python3

import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

joy_node_config_yaml =  os.path.join(
        get_package_share_directory('joystick_example'),
        'launch', 'joy_node_config.yaml'
    )
print("config.yaml location: ", joy_node_config_yaml);
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="joy",
            node_executable="joy_node",
	    parameters=[joy_node_config_yaml],
            output="screen"
        ),
    ])
