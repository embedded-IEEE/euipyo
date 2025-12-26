```
git clone https://github.com/embedded-IEEE/ros2_ws.git jetank_ws
```

사용법
===============================
알아서 빌드하고 setup.bash 실행하고
```
ros2 launch jetank_description gazebo.launch.py 
```
다른 터미널에서
```
ros2 run rqt_gui rqt_gui --ros-args -r __ns:=/jetank -r /tf:=tf -r /tf_static:=tf_static
```
rqt 나오면 plugins에서 Robot tools - Joint trajectory controller 열기<br/>
없으면
```
sudo apt install ros-humble-rqt* -y
rqt --force-discover
```

앞으로 수정사항
============================
현재 gazebo랑 rviz의 jetank가 연동 완료 했으니<br/>
이거랑 현실의 jetank랑 같은 코드로 돌릴 수 있게 코드 수정할 예정


rc카 시뮬 추가 명령어
============================
```
ros2 launch jetank_description gazebo.launch.py 
```

맵 생성 시 카토그래퍼 실행 명령어 (시뮬 실행 후 rf2o 메시지 확인 하고 켤 것)
```
ros2 launch rc_nav2_bringup bringup_slam_only.launch.py start_rf2o:=false use_sim_time:=true
```

맵 다 그린 후 맵 저장 명령어
```
ros2 run nav2_map_server map_saver_cli -f ~/maps/jetank_map
```

(-f 옵션 뒤는 저장하고 싶은 경로 넣을 것. 이 경로는 아래 nav2 자율주행 시 다시 사용해야 함)

자율주행 실행 명령어 (시뮬 다 켜지고 난 후 실행할 것.)

```
ros2 launch rc_nav2_bringup navigation.launch.py   use_sim_time:=True   slam:=False   map:=/home/ssafy/maps/jetank_map.yaml   use_namespace:=False   use_rviz:=True   cmd_vel_bridge_target:=/rc_car/cmd_vel

```

(map 옵션에는 저장했던 맵 파일 경로 넣어줄 것)

자율주행 방법

켜진 rviz에서, 2d pose estimate로 맵 상의 현재 로봇 위치를 클릭한 상태로 드래그해서 방향 지정. (정확하지 않아도 됨. 근처에만 찍으면 알아서 찾아감)
goal pose로 목표 위치 / 방향 지정
