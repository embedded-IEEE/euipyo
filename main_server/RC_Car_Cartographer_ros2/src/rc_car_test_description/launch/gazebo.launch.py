import os
from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # 패키지 경로
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robot       = get_package_share_directory('rc_car_test_description')

    # URDF(xacro) → robot_description
    robot_description_file = os.path.join(pkg_robot, 'urdf', 'rc_car_test.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # ros_gz_bridge 설정 파일
    ros_gz_bridge_config = os.path.join(pkg_robot, 'config', 'ros_gz_bridge_gazebo.yaml')

    # GZ 리소스 경로 추가
    env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            pkg_robot,
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ],
    )

    # 월드 파일 (GZ에서 로드)
    world_file = os.path.join(pkg_robot, 'worlds', 'empty_with_sensors.sdf')

    # Gazebo (gz_sim) 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            # -r: real-time, -v 4: verbose level
            "gz_args": f"-r -v 4 {world_file}"
        }.items()
    )

    # robot_state_publisher (URDF → TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
    )

    rsp_delay = TimerAction(
        period=3.0,
        actions=[robot_state_publisher]
    )
    # Gazebo 안에 로봇 스폰 (robot_description 기준)
    spawn_robot = TimerAction(
        period=3.0,  # gz 올라올 시간 약간 기다렸다가 스폰
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    "-topic", "/robot_description",
                    "-name", "rc_car_test",
                    "-allow_renaming", "false",
                    "-x", "0.0", "-y", "0.0", "-z", "0.32",
                    "-Y", "3.14159",  # Gazebo 모델 전방(-X)과 Nav2 전방(+X)을 맞추기 위해 본체를 180도 회전
                ],
                output='screen'
            )
        ]
    )

    # GZ ↔ ROS 브리지 (LaserScan 포함)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    # 🔧 LaserScan frame_id 덮어쓰기 노드
    #
    # 전제:
    #   - bridge에서 /lidar/scan  ->  /scan_raw 로 가져오고
    #   - 이 노드가 /scan_raw 구독 → /scan 퍼블리시
    #   - header.frame_id 를 lidar_link_1 로 고정
    #
    # cartographer_node 에서는 'scan' 토픽을 그대로 사용.
    scan_rewriter = Node(
        package='rc_car_test_description',
        executable='scan_frame_rewriter',
        name='scan_frame_rewriter',
        output='screen',
        parameters=[{'target_frame': 'lidar_link_1_back', 'use_sim_time': True}],
        remappings=[
            ('scan_in',  '/scan_raw'),  # 입력: bridge에서 나온 원래 스캔
            ('scan_out', '/scan'),      # 출력: frame_id 고친 스캔
        ]
    )

    # LiDAR 실제 빔이 바라보는 방향을 TF에 반영 (lidar_link_1 -> lidar_link_1_back, yaw 180도)
    lidar_back_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_back_tf',
        # x y z yaw pitch roll (yaw = 180deg)
        arguments=['0', '0', '0', '3.14159', '0', '0', 'lidar_link_1', 'lidar_link_1_back'],
        output='screen'
    )

    # 컨트롤러 스포너들
    js_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_controller"],
        output="screen"
    )

    wheel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_controller"],
        output="screen"
    )

    cmd_vel_converter = Node(
        package='rc_car_test_description',
        executable='twist_to_joint_cmd',
        name='twist_to_joint_cmd',
        output='screen',
        parameters=[{
            'drive_sign': 1.0,     # 휠 출력 부호 (필요 시 -1)
        }],
    )

    return LaunchDescription([
        env,
        gazebo,
        rsp_delay,
        spawn_robot,
        ros_gz_bridge,
        scan_rewriter,
        lidar_back_tf,
        js_broadcaster,
        steering_spawner,
        wheel_spawner,
        cmd_vel_converter,
    ])
