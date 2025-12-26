# RC_Car_SLAM-NAV2-with-Real-Robot

## 라즈베리파이 5(버전 상관 없음) Docker 컨테이너 실행

```
docker push ascroid/rpi5-ros2-humble-proj:ydlidar-real-robot
```

컨테이너에서 2가지 켤 것.

```
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

```
ros2 launch motorhat_controller motorhat_controller.launch.py
```

카토그래퍼로 맵 생성 시 명령어
```
ros2 launch rc_nav2_bringup bringup_slam_real.launch.py use_sim_time:=false
```

맵 생성 후 저장 명령어 (카토그래퍼 끄지 말고 켠 상태로 실행할 것.)

```
ros2 run nav2_map_server map_saver_cli -f ~/maps/jetank_map
```

생성한 맵으로 자율 주행 시 명령어
```
ros2 launch rc_nav2_bringup navigation_real.launch.py   use_sim_time:=false   slam:=false   map:=/home/ssafy/maps/jetank_map.yaml   use_rviz:=true

```
