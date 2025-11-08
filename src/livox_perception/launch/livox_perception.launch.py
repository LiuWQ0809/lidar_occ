from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_launch_arg = DeclareLaunchArgument(
        "config",
        default_value="livox_perception.yaml",
        description="Parameter file name located under the package config directory",
    )

    config_path = PathJoinSubstitution([
        FindPackageShare("livox_perception"),
        "config",
        LaunchConfiguration("config"),
    ])

    parameter_file = ParameterFile(config_path, allow_substs=True)

    perception_node = Node(
        package="livox_perception",
        executable="livox_perception_node",
        name="livox_perception_node",
        output="screen",
        parameters=[
            {"use_intra_process_comms": True},
            parameter_file,
        ],
    )

    return LaunchDescription([
        config_launch_arg,
        perception_node,
    ])
