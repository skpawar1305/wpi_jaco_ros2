
import os
import pathlib
import yaml
from launch.actions import LogInfo
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
import xacro


PACKAGE_NAME = 'jaco_moveit_config'


def generate_launch_description():
    launch_description_nodes = []
    package_dir = get_package_share_directory(PACKAGE_NAME)

    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'config', filename)).read_text()

    def load_yaml(filename):
        return yaml.safe_load(load_file(filename))

    # Check if moveit is installed
    if 'moveit' in get_packages_with_prefixes():
        # Configuration
        xacro_file = os.path.join(get_package_share_directory('jaco_description'), 'robots', 'standalone_arm.urdf.xacro')
        doc = xacro.process_file(xacro_file)
        description = {'robot_description': doc.toprettyxml(indent='  ')}

        description_semantic = {'robot_description_semantic': load_file('jaco.srdf')}
        description_kinematics = {'robot_description_kinematics': load_yaml('kinematics.yaml')}
        description_joint_limits = {'robot_description_planning': load_yaml('joint_limits.yaml')}
        sim_time = {'use_sim_time': False}

        # Rviz node
        rviz_config_file = os.path.join(package_dir, 'config', 'visualization.rviz')

        launch_description_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    description_joint_limits,
                    sim_time
                ],
            )
        )

        # Planning Configuration
        ompl_planning_pipeline_config = {
            "move_group": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
                "start_state_max_bounds_error": 0.1,
            }
        }
        # MoveIt2 node
        ompl_planning_yaml = load_yaml('ompl_planning.yaml')
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

        moveit_controllers = {
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            'moveit_simple_controller_manager': load_yaml('controllers.yaml')
        }

        launch_description_nodes.append(
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    moveit_controllers,
                    ompl_planning_pipeline_config,
                    description_joint_limits,
                    sim_time
                ],
                remappings=[('/joint_states', '/jaco_arm/joint_states')],
            )
        )
    else:
        launch_description_nodes.append(LogInfo(msg='"moveit" package is not installed, \
                                                please install it in order to run this demo.'))

    return LaunchDescription(launch_description_nodes)
