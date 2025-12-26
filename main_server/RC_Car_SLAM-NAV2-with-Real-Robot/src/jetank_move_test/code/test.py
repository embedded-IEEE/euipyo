#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys
import numpy as np

# --- 설정 ---
JOINT_NAMES = [
    'Revolute_BEARING',      # ID 1 (Base)
    'Revolute_SERVO_UPPER',  # ID 2 (Shoulder)
    'Revolute_ARM_LOW',      # ID 3 (Elbow)
    'Revolute_SERVO_TOP',    # ID 4 (Wrist Roll)
    'Revolute_MAGNETIC_BAR'  # ID 5 (Wrist Pitch)
]

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.pub = self.create_publisher(JointTrajectory, '/jetank/arm_controller/joint_trajectory', 10)
        self.current_angles = [0.0] * 5  # 모든 관절 0도로 시작

    def move_joint(self, joint_id, angle_deg):
        """
        joint_id: 1 ~ 5
        angle_deg: 목표 각도 (Degree)
        """
        # 인덱스는 0부터 시작하므로 -1
        idx = joint_id - 1
        
        # 상태 업데이트
        self.current_angles[idx] = float(angle_deg)

        # 메시지 생성
        msg = JointTrajectory()
        msg.header.stamp.sec = 0  # 즉시 실행
        msg.header.stamp.nanosec = 0
        msg.joint_names = JOINT_NAMES
        
        point = JointTrajectoryPoint()
        # Degree -> Radian 변환하여 전송
        point.positions = [np.radians(a) for a in self.current_angles]
        point.time_from_start = Duration(sec=1, nanosec=0) # 1초 동안 이동
        
        msg.points = [point]
        self.pub.publish(msg)
        
        print(f"\n[전송 완료] ID {joint_id}번 모터를 {angle_deg}도로 보냈습니다.")
        print(f"현재 상태(Deg): {self.current_angles}")

def main():
    rclpy.init()
    node = CalibrationNode()
    
    # 가제보가 켜질 때까지 잠깐 대기 겸 초기화 전송
    print(">> 초기화 중... 모든 관절을 0도로 맞춥니다.")
    node.move_joint(1, 0) 
    
    try:
        while True:
            print("\n" + "="*40)
            print("   [ 가제보 모터 방향/오프셋 캘리브레이션 도구 ]")
            print("="*40)
            print(" 1. Base (Revolute_BEARING)")
            print(" 2. Shoulder (Revolute_SERVO_UPPER)")
            print(" 3. Elbow (Revolute_ARM_LOW)")
            print(" 4. Wrist Roll (Revolute_SERVO_TOP)")
            print(" 5. Wrist Pitch (Revolute_MAGNETIC_BAR)")
            print(" Q. 종료")
            print("-" * 40)
            
            sel = input(">> 테스트할 모터 번호(1-5)를 입력하세요: ").strip().upper()
            if sel == 'Q': break
            
            if not sel.isdigit() or int(sel) < 1 or int(sel) > 5:
                print("!! 잘못된 번호입니다. 1~5 사이를 입력하세요.")
                continue
                
            try:
                ang = float(input(f">> ID {sel}번 모터의 목표 각도(도)를 입력하세요: "))
                node.move_joint(int(sel), ang)
            except ValueError:
                print("!! 숫자를 입력해주세요.")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()