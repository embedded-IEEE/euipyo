import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    rc_nav2_bringup_dir = get_package_share_directory('rc_nav2_bringup')
    rc_desc_dir = get_package_share_directory('rc_car_test_description')
    launch_dir = os.path.join(bringup_dir, 'launch')

    slam_input = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    cmd_vel_bridge_target = LaunchConfiguration('cmd_vel_bridge_target')
    cmd_vel_linear_scale = LaunchConfiguration('cmd_vel_linear_scale')
    cmd_vel_angular_scale = LaunchConfiguration('cmd_vel_angular_scale')
    use_display = LaunchConfiguration('use_display')
    scan_target_frame = LaunchConfiguration('scan_target_frame')
    scan_in_topic = LaunchConfiguration('scan_in_topic')
    scan_out_topic = LaunchConfiguration('scan_out_topic')
    start_rf2o = LaunchConfiguration('start_rf2o')
    rf2o_base_frame = LaunchConfiguration('rf2o_base_frame')
    rf2o_odom_frame = LaunchConfiguration('rf2o_odom_frame')
    rf2o_odom_topic = LaunchConfiguration('rf2o_odom_topic')
    rf2o_freq = LaunchConfiguration('rf2o_freq')
    enable_lidar_flip_tf = LaunchConfiguration('enable_lidar_flip_tf')
    lidar_frame = LaunchConfiguration('lidar_frame')
    lidar_flipped_frame = LaunchConfiguration('lidar_flipped_frame')
    lidar_flip_yaw = LaunchConfiguration('lidar_flip_yaw')

    slam = PythonExpression([
        '"', slam_input, '" == "true" or "', slam_input, '" == "True"'
    ])

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for Nav2 nodes (empty string for no namespace)')
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to Nav2')
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/maps/jetank_map.yaml'),
        description='Full path to map file to load')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(rc_nav2_bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz2')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup')
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')
    declare_cmd_vel_bridge_target_cmd = DeclareLaunchArgument(
        'cmd_vel_bridge_target',
        default_value='/rc_car/cmd_vel',
        description='Target topic to republish /cmd_vel to')
    declare_cmd_vel_linear_scale_cmd = DeclareLaunchArgument(
        'cmd_vel_linear_scale',
        default_value='1.0',
        description='Scale factor for linear velocity')
    declare_cmd_vel_angular_scale_cmd = DeclareLaunchArgument(
        'cmd_vel_angular_scale',
        default_value='1.0',
        description='Scale factor for angular velocity')
    declare_use_display_cmd = DeclareLaunchArgument(
        'use_display',
        default_value='True',
        description='Launch robot_state_publisher via display.launch.py')
    declare_scan_target_frame_cmd = DeclareLaunchArgument(
        'scan_target_frame',
        default_value='rc_car/lidar_link_1_back',
        description='Frame id to apply to LaserScan messages')
    declare_scan_in_topic_cmd = DeclareLaunchArgument(
        'scan_in_topic',
        default_value='/scan_raw',
        description='Input LaserScan topic name')
    declare_scan_out_topic_cmd = DeclareLaunchArgument(
        'scan_out_topic',
        default_value='/rc_car/scan',
        description='Output LaserScan topic name')
    declare_start_rf2o_cmd = DeclareLaunchArgument(
        'start_rf2o',
        default_value='True',
        description='Start RF2O in this launch')
    declare_rf2o_base_frame_cmd = DeclareLaunchArgument(
        'rf2o_base_frame',
        default_value='rc_car/base_link_rot',
        description='RF2O base_frame_id')
    declare_rf2o_odom_frame_cmd = DeclareLaunchArgument(
        'rf2o_odom_frame',
        default_value='rc_car/odom_rf2o',
        description='RF2O odom_frame_id')
    declare_rf2o_odom_topic_cmd = DeclareLaunchArgument(
        'rf2o_odom_topic',
        default_value='/rc_car/odom_rf2o',
        description='RF2O odom topic name')
    declare_rf2o_freq_cmd = DeclareLaunchArgument(
        'rf2o_freq',
        default_value='10.0',
        description='RF2O processing frequency (Hz)')
    declare_enable_lidar_flip_tf_cmd = DeclareLaunchArgument(
        'enable_lidar_flip_tf',
        default_value='True',
        description='Publish static TF for lidar_link_1_back')
    declare_lidar_frame_cmd = DeclareLaunchArgument(
        'lidar_frame',
        default_value='rc_car/lidar_link_1',
        description='Original LiDAR frame')
    declare_lidar_flipped_frame_cmd = DeclareLaunchArgument(
        'lidar_flipped_frame',
        default_value='rc_car/lidar_link_1_back',
        description='Corrected LiDAR frame (yaw 180deg)')
    declare_lidar_flip_yaw_cmd = DeclareLaunchArgument(
        'lidar_flip_yaw',
        default_value='3.14159',
        description='Yaw (rad) for LiDAR frame correction')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_rviz': use_rviz,
            'use_composition': use_composition,
            'namespace': namespace,
            'use_namespace': use_namespace
        }.items())

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rc_desc_dir, 'launch', 'display.launch.py')
        ),
        condition=IfCondition(use_display),
        launch_arguments={
            'gui': 'False',
            'use_rviz': 'False',
        }.items()
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
            os.path.join(rc_nav2_bringup_dir, 'launch', 'rf2o.launch.py')),
        condition=IfCondition(start_rf2o),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'scan_topic': scan_out_topic,
            'base_frame_id': rf2o_base_frame,
            'odom_frame_id': rf2o_odom_frame,
            'odom_topic': rf2o_odom_topic,
            'freq': rf2o_freq,
        }.items()
    )

    scan_rewriter = Node(
        package='rc_car_test_description',
        executable='scan_frame_rewriter',
        name='scan_frame_rewriter',
        output='screen',
        parameters=[{
            'target_frame': scan_target_frame,
            'scan_in_topic': scan_in_topic,
            'scan_out_topic': scan_out_topic,
            'force_stamp_now': True,
            'use_sim_time': use_sim_time,
        }],
    )

    cmd_vel_bridge = Node(
        package='rc_nav2_bringup',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'target_topic': cmd_vel_bridge_target,
            'linear_scale': cmd_vel_linear_scale,
            'angular_scale': cmd_vel_angular_scale,
        }],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_cmd_vel_bridge_target_cmd)
    ld.add_action(declare_cmd_vel_linear_scale_cmd)
    ld.add_action(declare_cmd_vel_angular_scale_cmd)
    ld.add_action(declare_use_display_cmd)
    ld.add_action(declare_scan_target_frame_cmd)
    ld.add_action(declare_scan_in_topic_cmd)
    ld.add_action(declare_scan_out_topic_cmd)
    ld.add_action(declare_start_rf2o_cmd)
    ld.add_action(declare_rf2o_base_frame_cmd)
    ld.add_action(declare_rf2o_odom_frame_cmd)
    ld.add_action(declare_rf2o_odom_topic_cmd)
    ld.add_action(declare_rf2o_freq_cmd)
    ld.add_action(declare_enable_lidar_flip_tf_cmd)
    ld.add_action(declare_lidar_frame_cmd)
    ld.add_action(declare_lidar_flipped_frame_cmd)
    ld.add_action(declare_lidar_flip_yaw_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(display_launch)
    ld.add_action(lidar_flip_tf)
    ld.add_action(scan_rewriter)
    ld.add_action(rf2o_launch)
    ld.add_action(cmd_vel_bridge)

    return ld
