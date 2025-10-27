"""Launch GUI (simulation + GUI) and the custom decision node only.
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare

# reuse helper from crossing_gui_ros2/launch/scripts if present
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'crossing_gui_ros2', 'launch', 'scripts'))
try:
    from anchor_and_alias_yaml_support import anchor_and_alias_yaml_support
except Exception:
    # fallback: the helper lives in the crossing_gui_ros2 package; try to import from there
    try:
        sys.path.append(os.path.join(get_package_share_directory('crossing_gui_ros2'), 'launch', 'scripts'))
        from anchor_and_alias_yaml_support import anchor_and_alias_yaml_support
    except Exception:
        anchor_and_alias_yaml_support = None


def generate_launch_description():

    ld = LaunchDescription()

    # Choose the parameter set:
    config_file_name = "params.yaml"
    corrected_config_file_name = "corrected_params.yaml"

    config_folder_path = os.path.join(get_package_share_directory('crossing_gui_ros2'), 'config')
    config_file_path = os.path.join(config_folder_path, config_file_name)
    corrected_config_file_path = os.path.join(config_folder_path, corrected_config_file_name)

    if anchor_and_alias_yaml_support is not None:
        try:
            anchor_and_alias_yaml_support(input_file_path=config_file_path, 
                                          output_file_path=corrected_config_file_path)
        except Exception:
            corrected_config_file_path = config_file_path
    else:
        corrected_config_file_path = config_file_path

    # simulation node (from crossing_gui_ros2)
    simulation_node = Node(
        package='crossing_gui_ros2',
        namespace=None,
        executable='simulation_node',
        name='simulation_node',
        parameters=[corrected_config_file_path]
    )

    simulation_gui_node = Node(
        package='crossing_gui_ros2',
        namespace=None,
        executable='simulation_gui_node',
        name='simulation_gui_node',
        parameters=[corrected_config_file_path]
    )

    # reuse control launch descriptions from crossing_gui_ros2
    control_vehicle_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crossing_gui_ros2'), 'launch', 'start_vehicle_control_node_launch.py'
            ])
        ]),
        launch_arguments={
            'config_file_path': corrected_config_file_path,
            'controller_type': 'control_vehicle_mpc'
        }.items()
    )

    control_pedestrian_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crossing_gui_ros2'), 'launch', 'start_pedestrian_control_node_launch.py'
            ])
        ]),
        launch_arguments={
            'config_file_path': corrected_config_file_path,
            'controller_type': 'control_pedestrian_keyboard'
        }.items()
    )

    # logging node from crossing_gui_ros2
    log_node = Node(
        package='crossing_gui_ros2',
        namespace=None,
        executable='log',
        name='log_node',
        parameters=[corrected_config_file_path]
    )

    # the custom decision node (replacement of decision_making_node)
    custom_decision_node = Node(
        package='decision_making_package',
        namespace=None,
        executable='custom_decision_making_node',
        name='decision_making_node',
        parameters=[corrected_config_file_path]
    )

    # start vehicle & pedestrian control
    ld.add_action(control_vehicle_node)
    ld.add_action(control_pedestrian_node)

    ld.add_action(simulation_node)
    ld.add_action(simulation_gui_node)

    # decision node 
    ld.add_action(custom_decision_node)

    # log node
    ld.add_action(log_node)

    return ld
