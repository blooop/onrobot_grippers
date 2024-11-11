import os
from ament_index.common import get_package_index
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OmitOutput
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch file parameters
        DeclareLaunchArgument(
            'paused',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'recording',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='125'
        ),

        # Start up simulated world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_index().find('gazebo_ros'), 'launch', 'empty_world.launch.py')
            ),
            launch_arguments={
                'paused': LaunchConfiguration('paused'),
                'debug': LaunchConfiguration('debug'),
                'recording': LaunchConfiguration('recording'),
                'gui': LaunchConfiguration('gui')
            }
        ),

        # Start up robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        ),

        # Send robot urdf to param server
        DeclareLaunchArgument(
            'robot_description',
            default_value=os.path.join(ThisLaunchFileDir(), '..', '..', 'onrobot_rg_description', 'urdf', 'onrobot_rg2_model.xacro')
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_urdf',
            output='screen',
            arguments=[
                '-urdf -param robot_description -model onrobot_rg2'
            ]
        ),

        # Start up controllers
        ExecuteProcess(
            cmd=['rosparam', 'load', os.path.join(ThisLaunchFileDir(), '..', '..', 'onrobot_rg_control', 'config', 'rg2_control_params.yaml')],
            name='controller_spawner',
            output='screen',
            shell=True,
            process_target=OmitOutput()
        ),
    ])
