from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare('elevation_mapping_demos').find('elevation_mapping_demos')
    rviz_config_path = os.path.join(pkg_share, 'launch', 'rviz_config.rviz')
    print(f"RViz config path: {rviz_config_path}")  # 打印路径以验证

    use_rviz = LaunchConfiguration('start_rviz')
    realsense_share = FindPackageShare('realsense2_camera')
    demos_share = FindPackageShare('elevation_mapping_demos')
    rtabmap_share = FindPackageShare('rtabmap_launch') # 新增：rtabmap 路径

    use_realsense = LaunchConfiguration('use_realsense')
    use_rtabmap = LaunchConfiguration('use_rtabmap') # 新增：rtabmap 开关
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
            'use_rtabmap',
            default_value='true',
            description='Launch RTAB-Map for Visual Odometry and SLAM.'
        ),
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/camera/camera/depth/color/points',
            description='Point cloud topic consumed by elevation_mapping.'
        ),
        DeclareLaunchArgument(
            'map_frame',
            # 重要修改：动态建图建议使用 odom 作为高程图的全局坐标系，避免闭环导致地图撕裂
            default_value='odom', 
            description='Global map frame for elevation map.'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='camera_link',
            description='Robot base frame used by elevation_mapping.'
        ),
        DeclareLaunchArgument(
            'publish_map_base_tf',
            # 重要修改：由于 rtabmap 会发布 TF，这里默认设为 false
            default_value='false', 
            description='Publish a static transform map -> base_frame (demo only).'
        ),
        DeclareLaunchArgument(
            'start_rviz', # 改名
            default_value='true',
            description='Whether to start our custom RViz'
        ),
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
            'pointcloud.enable': 'true',
            'pointcloud.ordered_pc': 'true',
            'align_depth.enable': 'true',
            'global_time_enabled': 'true',
        }.items(),
        condition=IfCondition(use_realsense),
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_share, 'launch', 'rtabmap.launch.py'])
        ),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start --Params "Rtabmap/DetectionRate=5"',
            'frame_id': base_frame,
            'visual_odometry': 'true',
            'icp_odometry': 'false',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'approx_sync': 'true',
            'qos': '1',
            'publish_tf_map': 'true',
            'rviz': 'false',
        }.items(),
        condition=IfCondition(use_rtabmap),
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
                # 这里保持为空，elevation_mapping 会自动通过侦听 TF (odom -> camera_link) 来获取位姿
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

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_main', # 也可以改个名防止节点名冲突
            arguments=['-d', rviz_config_path],
            condition=IfCondition(use_rviz), # 使用新的变量名
            output='screen'
    )

    return LaunchDescription(declarations + [
        realsense_launch,
        rtabmap_launch,           # 插入 RTAB-Map
        elevation_mapping_node,
        static_tf,
        rviz_node
    ])