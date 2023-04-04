import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

import xacro
import yaml

configurable_parameters = [
    {'name': 'arm_name',              'default': "jaco"},
]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def launch_setup(context, *args, **kwargs):
    _config_file = os.path.join(
        get_package_share_directory('wpi_jaco_wrapper'),
        'config',
        'jaco.yaml'
    )
    params_from_file = yaml_to_dict(_config_file)

    robot_type = LaunchConfiguration("arm_name").perform(context)
    wpi_jaco_wrapper = Node(
        package='wpi_jaco_wrapper',
        executable='jaco_arm_trajectory_node',
        parameters=[set_configurable_parameters(configurable_parameters), params_from_file],
        output='screen',
    )
    
    # xacro_file = os.path.join(get_package_share_directory(robot_type + '_description'), 'robots', 'standalone_arm.urdf.xacro')
    # doc = xacro.process_file(xacro_file)
    # robot_desc = doc.toprettyxml(indent='  ')
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     remappings=[('joint_states', '/jaco_arm/joint_states')],
    #     output='screen',
    #     parameters=[{'robot_description': robot_desc},],
    # )
    
    return [wpi_jaco_wrapper]#, robot_state_publisher]

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function = launch_setup)
    ])
