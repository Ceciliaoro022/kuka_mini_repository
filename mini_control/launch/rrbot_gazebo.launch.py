from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='mini_control').find('mini_control')
    
    urdf_file = PathJoinSubstitution([pkg_share, 'description', 'urdf', 'rrbot.urdf.xacro'])
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            urdf_file
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}
    

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rrbot'
        ],
        output='screen'
    )
    
    # Load controllers
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    load_joint_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_effort_controller', '--controller-manager', '/controller_manager'],
    )
    
    load_joint_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_velocity_controller', '--controller-manager', '/controller_manager'],
    )

    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )
    

    load_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                load_joint_state_broadcaster,
                load_joint_effort_controller,
                #load_joint_velocity_controller
                #load_joint_trajectory_controller
            ],
        )
    )
    

    return LaunchDescription([
  
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        gazebo,
        spawn_entity,
        load_controllers_after_spawn
    ])
