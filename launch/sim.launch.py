import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, Shutdown, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('golf_cart_sim')
    config = LaunchConfiguration('config').perform(context)
    with open(config, encoding='utf-8') as stream:
        limits = yaml.safe_load(stream)
    robot = xacro.process_file(os.path.join(share, 'urdf/golf_cart.urdf.xacro'),
                               mappings={'config_file': config, 'detailed_visuals': LaunchConfiguration('detailed_visuals').perform(context)}).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}],
             remappings=[('joint_states', '/golf/joint_states')]),
        ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                            '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
                       output='screen', on_exit=Shutdown()),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'golf_cart', '-topic', 'robot_description',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')], output='screen'),
        ExecuteProcess(cmd=['gzclient'], condition=IfCondition(LaunchConfiguration('gui')),
                       on_exit=Shutdown()),
        Node(package='golf_cart_sim', executable='control_ui', output='screen',
             parameters=[{'max_speed': float(limits['max_speed']),
                          'max_steer': float(limits['max_steer'])}],
             condition=IfCondition(LaunchConfiguration('manual_ui'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('golf_cart_sim')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('manual_ui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('detailed_visuals', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config/vehicle.yaml')),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds/flat.world')),
        DeclareLaunchArgument('x', default_value='0'), DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='0.30'), DeclareLaunchArgument('yaw', default_value='0'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.dirname(share)
                               + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', os.path.join(get_package_prefix('golf_cart_sim'), 'lib')
                               + os.pathsep + os.environ.get('GAZEBO_PLUGIN_PATH', '')),
        OpaqueFunction(function=setup),
    ])
