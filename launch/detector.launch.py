# synapse_vision_traffic_detector/launch/detector.launch.py
#
# This launch file starts the traffic_sign_detector_node
# and loads its parameters from the params.yaml file.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package share directory for 'synapse_vision_traffic_detector'
    pkg_share_dir = get_package_share_directory('synapse_vision_traffic_detector')

    # Construct the full path to the parameters YAML file
    params_file_path = os.path.join(pkg_share_dir, 'config', 'params.yaml')

    # Declare a launch argument for the parameters file path.
    # This allows users to optionally override the default params.yaml path from the command line.
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Path to the YAML file with node parameters'
    )

    # Define the traffic_sign_detector_node.
    # This specifies how the ROS 2 executable should be launched.
    traffic_sign_detector_node = Node(
        package='synapse_vision_traffic_detector',  # The name of your ROS 2 package.
        executable='traffic_sign_detector_node',    # The name of the executable binary
                                                    # defined in your CMakeLists.txt (add_executable).
        name='traffic_sign_detector_node',          # The name of this specific node instance in the ROS graph.
        output='screen',                            # Directs the node's console output to the terminal.
        parameters=[LaunchConfiguration('params_file')], # Loads parameters from the specified YAML file.
        # Optional: Add remappings here if your image input topic is different from 'camera/image_raw'.
        # For example:
        # remappings=[
        #     ('camera/image_raw', '/my_robot/camera/image')
        # ]
    )

    # Create the LaunchDescription object and add all defined actions to it.
    # This object is returned by the generate_launch_description function
    # and is what ROS 2 executes.
    return LaunchDescription([
        declare_params_file_cmd,
        traffic_sign_detector_node
    ])
