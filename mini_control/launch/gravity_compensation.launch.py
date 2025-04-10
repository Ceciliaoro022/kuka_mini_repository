from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    default_urdf_path = '/home/amir/ros2_ws/src/mini_control/description/urdf/rrbot.urdf'
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf_path,
    )
      
    gravity_compensation = Node(
        package='mini_control',
        executable='gravity_compensation',
        name='gravity_compensation',
        output='screen',
        parameters=[{
            'urdf_path': LaunchConfiguration('urdf_path')
        }]
    )
    
    return LaunchDescription([
        urdf_path_arg,
        gravity_compensation
    ])