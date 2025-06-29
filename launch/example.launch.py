import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    bento_drive = Node(
        package='cv_color_mapper',
        executable='color_map_node',
        name='color_drive_node',
        parameters=[ {"colormap": "jet"} ],
        namespace=EnvironmentVariable( 'EDU_ROBOT_NAMESPACE', default_value="bento" ),
        output='screen'
    )

    return LaunchDescription([
        bento_drive
    ])
