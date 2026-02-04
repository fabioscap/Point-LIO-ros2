from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('point_lio'),
        'config',
        'ouster64.yaml'
    ])

    return LaunchDescription([
        Node(
            package='point_lio',
            executable='pointlio_mapping',
            name='laserMapping',
            parameters=[config],
            output='screen'
        )
    ])
