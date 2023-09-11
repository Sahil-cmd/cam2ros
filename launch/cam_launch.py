"""
Copyright (c) 2023 Sahil Panjwani

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
       
        launch.actions.DeclareLaunchArgument(
            'params',
            default_value=PathJoinSubstitution([
                launch.substitutions.ThisLaunchFileDir(), 
                'config', 
                'cam_params.yaml'
            ]),
            description='Path to the config file for the CamPublisher node'
        ),

        Node(
            package='camera_publisher',
            executable='cam_node',
            name='camera_publisher',
            output='screen',
            parameters=[launch.substitutions.LaunchConfiguration('params')],
            remappings=None  
        ),
    ])
