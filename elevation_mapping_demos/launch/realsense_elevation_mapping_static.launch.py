from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    realsense_share = FindPackageShare('realsense2_camera')
    demos_share = FindPackageShare('elevation_mapping_demos')

    use_realsense = LaunchConfiguration('use_realsense')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')
    map_frame = LaunchConfiguration('map_frame')
    base_frame = LaunchConfiguration('base_frame')
    publish_map_base_tf = LaunchConfiguration('publish_map_base_tf')

    declarations = [
        DeclareLaunchArgument(
            'use_realsense',
            default_value='true',
            description='Launch Intel RealSense camera driver.'
        ),
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/camera/camera/depth/color/points',
            description='Point cloud topic consumed by elevation_mapping.'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Global map frame for elevation map.'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='camera_link',
            description='Robot base frame used by elevation_mapping.'
        ),
        DeclareLaunchArgument(
            'publish_map_base_tf',
            default_value='true',
            description='Publish a static transform map -> base_frame (demo only).'
        ),
    ]

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([realsense_share, 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
            'enable_depth': 'true',
            'enable_color': 'true',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
        }.items(),
        condition=IfCondition(use_realsense),
    )

    elevation_mapping_node = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='elevation_mapping',
        output='screen',
        parameters=[
            {
                'map_frame_id': map_frame,
                'robot_base_frame_id': base_frame,
                'robot_pose_with_covariance_topic': '',
                'robot_pose_cache_size': 200,
                'track_point_frame_id': base_frame,
                'track_point_x': 0.0,
                'track_point_y': 0.0,
                'track_point_z': 0.0,
                'min_update_rate': 2.0,
                'time_tolerance': 1.0,
                'time_offset_for_point_cloud': 0.0,
                'point_cloud_topic': point_cloud_topic,
                'length_in_x': 2.5,
                'length_in_y': 2.5,
                'position_x': 0.0,
                'position_y': 0.0,
                'resolution': 0.02,
                'min_variance': 0.000009,
                'max_variance': 0.01,
                'mahalanobis_distance_threshold': 2.5,
                'multi_height_noise': 0.0000009,
                'fused_map_publishing_rate': 10.0,
                'enable_visibility_cleanup': False,
                'sensor_processor/cutoff_min_depth': 0.01,
                'sensor_processor/cutoff_max_depth': 3.25,
                'sensor_processor/normal_factor_a': 0.000611,
                'sensor_processor/normal_factor_b': 0.003587,
                'sensor_processor/normal_factor_c': 0.3515,
                'sensor_processor/normal_factor_d': 0.0,
                'sensor_processor/normal_factor_e': 1.0,
                'sensor_processor/lateral_factor': 0.01576,
            }
        ],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', map_frame, base_frame],
        condition=IfCondition(publish_map_base_tf),
    )

    return LaunchDescription(declarations + [
        realsense_launch,
        elevation_mapping_node,
        static_tf,
    ])
