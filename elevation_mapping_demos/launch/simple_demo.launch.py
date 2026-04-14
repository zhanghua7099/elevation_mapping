from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_demos = get_package_share_directory('elevation_mapping_demos')

    elevation_mapping_node = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='elevation_mapping',
        output='screen',
        parameters=[
            os.path.join(pkg_demos, 'config', 'robots', 'simple_demo_robot.yaml'),
            os.path.join(pkg_demos, 'config', 'elevation_maps', 'simple_demo_map.yaml'),
            os.path.join(pkg_demos, 'config', 'postprocessing', 'postprocessor_pipeline.yaml'),
        ]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot',
        arguments=['2.0', '6.0', '0', '0', '0.0', '0', 'map', 'base']
    )

    return LaunchDescription([
        elevation_mapping_node,
        static_tf,
    ])
