# RC_Car_Cartographer_ros2
cartographer SLAM with pi5 rc car
https://github.com/embedded-IEEE/project_IEEE/issues/51

# 실행할 명령어 모음

## Cartographer로 맵 그리기
- Cartographer + gazebo
  - ``` bash
    ros2 launch rc_nav2_bringup bringup_all.launch.py use_gazebo:=True use_sim_time:=True scan_topic:=/scan rf2o_freq:=5.0
    ```
    
- 명령어 실행 후 켜지는 rviz에서, 왼쪽 아래 add를 눌러서, Map 추가 후 topic 칸에 /map 추가할 것.

## Nav2로 자율주행하기

- gazebo 시뮬레이터 실행
  - ``` bash
    ros2 launch rc_car_test_description gazebo.launch.py 
    ```
- RF2O 패키지 실행
  - ```bash
    ros2 launch rc_nav2_bringup rf2o.launch.py use_sim_time:=True scan_topic:=/scan rf2o_freq:=5.0
    ```
- Nav2 실행
  - ``` bash
    ros2 launch rc_nav2_bringup navigation.launch.py   slam:=False map:=$HOME/maps/my_map.yaml   use_sim_time:=True use_composition:=False use_rviz:=True 
    ```
