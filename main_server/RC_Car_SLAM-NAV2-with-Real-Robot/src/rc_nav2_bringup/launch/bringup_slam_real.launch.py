import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rc_nav2_bringup')
    pkg_desc = get_package_share_directory('rc_car_test_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic_in = LaunchConfiguration('scan_topic_in')
    scan_topic = LaunchConfiguration('scan_topic')
    scan_target_frame = LaunchConfiguration('scan_target_frame')
    rf2o_base_frame = LaunchConfiguration('rf2o_base_frame')
    rf2o_odom_frame = LaunchConfiguration('rf2o_odom_frame')
    rf2o_odom_topic = LaunchConfiguration('rf2o_odom_topic')
    rf2o_freq = LaunchConfiguration('rf2o_freq')
    use_rviz = LaunchConfiguration('use_rviz')
    start_rf2o = LaunchConfiguration('start_rf2o')
    carto_odom_topic = LaunchConfiguration('carto_odom_topic')
    enable_lidar_flip_tf = LaunchConfiguration('enable_lidar_flip_tf')
    lidar_frame = LaunchConfiguration('lidar_frame')
    lidar_flipped_frame = LaunchConfiguration('lidar_flipped_frame')
    lidar_flip_yaw = LaunchConfiguration('lidar_flip_yaw')

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'gui': 'False',
            'use_rviz': 'False',
        }.items()
    )

    scan_rewriter = Node(
        package='rc_car_test_description',
        executable='scan_frame_rewriter',
        name='scan_frame_rewriter',
        output='screen',
        parameters=[{
            'target_frame': scan_target_frame,
            'scan_in_topic': scan_topic_in,
            'scan_out_topic': scan_topic,
            'force_stamp_now': True,
            'use_sim_time': use_sim_time,
        }],
    )

    lidar_flip_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_flip_tf',
        arguments=['0', '0', '0', lidar_flip_yaw, '0', '0', lidar_frame, lidar_flipped_frame],
        output='screen',
        condition=IfCondition(enable_lidar_flip_tf),
    )

    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rf2o.launch.py')),
        condition=IfCondition(start_rf2o),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'scan_topic': scan_topic,
            'base_frame_id': rf2o_base_frame,
            'odom_frame_id': rf2o_odom_frame,
            'odom_topic': rf2o_odom_topic,
            'freq': rf2o_freq,
        }.items()
    )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bringup_cartographer.launch.py')),
        launch_arguments={
            'use_gazebo': 'False',
            'use_display': 'False',
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'scan_topic': scan_topic,
            'odom_topic': carto_odom_topic,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'scan_topic_in',
            default_value='/scan_raw',
            description='Laser scan input topic (raw)'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/rc_car/scan',
            description='Laser scan topic'),
        DeclareLaunchArgument(
            'scan_target_frame',
            default_value='rc_car/lidar_link_1_back',
            description='Frame id to apply to LaserScan messages'),
        DeclareLaunchArgument(
            'rf2o_base_frame',
            default_value='rc_car/base_link_rot',
            description='RF2O base_frame_id'),
        DeclareLaunchArgument(
            'rf2o_odom_frame',
            default_value='rc_car/odom_rf2o',
            description='RF2O odom_frame_id'),
        DeclareLaunchArgument(
            'rf2o_odom_topic',
            default_value='/rc_car/odom_rf2o',
            description='RF2O odom topic name'),
        DeclareLaunchArgument(
            'carto_odom_topic',
            default_value='/rc_car/odom_rf2o',
            description='Cartographer odom topic name'),
        DeclareLaunchArgument(
            'enable_lidar_flip_tf',
            default_value='True',
            description='Publish static TF for lidar_link_1_back'),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='rc_car/lidar_link_1',
            description='Original LiDAR frame'),
        DeclareLaunchArgument(
            'lidar_flipped_frame',
            default_value='rc_car/lidar_link_1_back',
            description='Corrected LiDAR frame (yaw 180deg)'),
        DeclareLaunchArgument(
            'lidar_flip_yaw',
            default_value='3.14159',
            description='Yaw (rad) for LiDAR frame correction'),
        DeclareLaunchArgument(
            'rf2o_freq',
            default_value='10.0',
            description='RF2O 처리 주기 (Hz)'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Whether to launch RViz (map can be viewed in existing RViz)'),
        DeclareLaunchArgument(
            'start_rf2o',
            default_value='True',
            description='RF2O를 이 런치에서 켤지 여부 (Gazebo에서 이미 구동 중이면 False)'),
        display_launch,
        lidar_flip_tf,
        scan_rewriter,
        rf2o_launch,
        cartographer_launch,
    ])
