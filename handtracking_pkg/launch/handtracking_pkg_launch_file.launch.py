from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='handtracking_pkg',
            executable='simple_publisher',
            output='screen'
        ),
        Node(
            package='handtracking_pkg',
            executable='lidar',
            output='screen'
        ),
    ])
    ld = LaunchDescription()

    handtracking_node =Node(
            package='handtracking_pkg',
            executable='simple_publisher',
            output='screen')

    lidar_node =Node(
        package='handtracking_pkg',
        executable='lidar',
        output='screen')


    ld.add_action(handtracking_node)
    ld.add_action(lidar_node)

    return ld
