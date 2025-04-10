#Just for trial, it does not control the KUKA, just the mini. It is just to see if I can spawn both robots in Gazebo

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the combined URDF xacro file
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("kuka_mini"), "kuka_mini_urdf", "kuka_mini_combined.urdf.xacro"]
    )

    # Get the actual file path for gazebo.launch.py
    gazebo_launch_path = os.path.join(get_package_share_directory("gazebo_ros"),
                                        "launch", "gazebo.launch.py")

    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    # Robot State Publisher node with processed xacro file
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
    )

    # Launch the ros2_control node (controller manager)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
         parameters=[
        {'robot_description': Command(['xacro ', urdf_file])},
        PathJoinSubstitution([FindPackageShare("iiwa_description"), "config", "iiwa_controllers.yaml"]), #iiwa_description is inside iiwa_ros2
        PathJoinSubstitution([FindPackageShare("mini_control"), "config", "controllers.yaml"]),
        ],
        output="screen",
    )

    # Spawn the robot in Gazebo with a delay to ensure Gazebo has started
    spawn_entity = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "/robot_description", "-entity", "my_robot"],
            output="screen"
        )]
    )

    # RViz node for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("iiwa_description"), "rviz", "rviz_config.rviz"]
        )],
        output="screen"
    )

    # Controller spawner nodes for 2r_effort_controller and iiwa_controller
    iiwa_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["iiwa_arm_controller"],
        output="screen"
    )

    mini_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_effort_controller"],
        output="screen"
    )

    state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    # Delay controller spawners to ensure the controller manager is up
    controller_spawner = TimerAction(
        period=7.0,  # Adjust delay as necessary
        actions=[iiwa_arm_controller_spawner, mini_controller_spawner, state_broadcaster_spawner],
    )

    return LaunchDescription([
        gazebo_launch,
        rsp_node,
        ros2_control_node,  # Make sure this is launched so the service is available
        spawn_entity,
        rviz_node,
        controller_spawner
    ])