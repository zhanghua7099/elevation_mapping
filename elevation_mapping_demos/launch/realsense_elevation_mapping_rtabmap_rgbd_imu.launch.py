from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('elevation_mapping_demos').find('elevation_mapping_demos')
    rviz_config_path = os.path.join(pkg_share, 'launch', 'rviz_config.rviz')

    realsense_share = FindPackageShare('realsense2_camera')
    rtabmap_share = FindPackageShare('rtabmap_launch')

    use_realsense = LaunchConfiguration('use_realsense')
    use_rtabmap = LaunchConfiguration('use_rtabmap')
    use_rviz = LaunchConfiguration('start_rviz')

    point_cloud_topic = LaunchConfiguration('point_cloud_topic')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    publish_map_base_tf = LaunchConfiguration('publish_map_base_tf')

    declarations = [
        DeclareLaunchArgument('use_realsense', default_value='true'),
        DeclareLaunchArgument('use_rtabmap', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),

        DeclareLaunchArgument('point_cloud_topic', default_value='/camera/camera/depth/color/points'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('imu_topic', default_value='/camera/camera/imu'),

        DeclareLaunchArgument('map_frame', default_value='odom'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='camera_link'),
        DeclareLaunchArgument('publish_map_base_tf', default_value='false'),
    ]

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([realsense_share, 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',

            'pointcloud.enable': 'true',
            'pointcloud.ordered_pc': 'true',

            # RealSense IMU, for D435i/D455 etc.
            # unite_imu_method: 0=none, 1=copy, 2=linear_interpolation
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'gyro_fps': '200',
            'accel_fps': '63',

            'global_time_enabled': 'true',
            'publish_tf': 'true',
        }.items(),
        condition=IfCondition(use_realsense),
    )

    # RealSense IMU -> orientation IMU
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', '/imu/data'),
        ],
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_share, 'launch', 'rtabmap.launch.py'])
        ),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start --ros-args -p Rtabmap/DetectionRate:=5',

            'frame_id': base_frame,
            'odom_frame_id': odom_frame,
            'map_frame_id': 'map',
            'publish_tf_map': 'true',
            'publish_tf_odom': 'true',

            'visual_odometry': 'true',
            'icp_odometry': 'false',

            'rgb_topic': rgb_topic,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'approx_sync': 'true',
            'qos': '1',

            # RTAB-Map IMU mode
	    'imu_topic': '/imu/data',
	    'wait_imu_to_init': 'true',

            'rviz': 'false',
        }.items(),
        condition=IfCondition(use_rtabmap),
    )

    elevation_mapping_node = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='elevation_mapping',
        output='screen',
        parameters=[{
            'map_frame_id': map_frame,
            'robot_base_frame_id': base_frame,
            'robot_pose_with_covariance_topic': '',
            'robot_pose_cache_size': 200,
            'track_point_frame_id': base_frame,
            'track_point_x': 0.0,
            'track_point_y': 0.0,
            'track_point_z': 0.0,
            'min_update_rate': 10.0,
            'time_tolerance': 1.0,
            'time_offset_for_point_cloud': 0.0,
            'point_cloud_topic': point_cloud_topic,
            'length_in_x': 3.5,
            'length_in_y': 3.5,
            'position_x': 0.0,
            'position_y': 0.0,
            'resolution': 0.05,
            'min_variance': 0.000009,
            'max_variance': 0.01,
            'mahalanobis_distance_threshold': 2.5,
            'multi_height_noise': 0.0000009,
            'fused_map_publishing_rate': 10.0,
            'enable_visibility_cleanup': True,
            'sensor_processor/cutoff_min_depth': 0.05,
            'sensor_processor/cutoff_max_depth': 3.25,
            'sensor_processor/normal_factor_a': 0.000611,
            'sensor_processor/normal_factor_b': 0.003587,
            'sensor_processor/normal_factor_c': 0.3515,
            'sensor_processor/normal_factor_d': 0.0,
            'sensor_processor/normal_factor_e': 1.0,
            'sensor_processor/lateral_factor': 0.01576,
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', map_frame, base_frame],
        condition=IfCondition(publish_map_base_tf),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_main',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription(declarations + [
        realsense_launch,
        rtabmap_launch,
        elevation_mapping_node,
        imu_filter_node,
        static_tf,
        rviz_node,
    ])
