#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from motorhat_controller.drivers.Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
from motorhat_controller.drivers.Raspi_PWM_Servo_Driver import PWM
import math


WHEEL_BASE = 0.30
MAX_STEER = 0.6       # rad
MIN_SPEED = 0.05      # m/s (0 근처 보호)
MAX_LINEAR = 0.6   # m/s, 차량이 낼 수 있는 최고 속도
MIN_DEADBAND = 0.02  # m/s, 너무 작은 값은 정지 처리


class MotorHatController(Node):
    def __init__(self):
        super().__init__('motorhat_controller')

        # MotorHAT init
        self.mh = Raspi_MotorHAT(addr=0x6F)
        self.motor = self.mh.getMotor(2)       # DC motor on M2
        self.servo = PWM(0x6F)                 # Servo driver
        self.servo.setPWMFreq(60)

        # Steering constants
        self.SERVO_LEFT  = 216
        self.SERVO_RIGHT = 440
        self.SERVO_CENTER = 328

        # Max PWM for motor
        self.MOTOR_MAX = 255

        self.subscription = self.create_subscription(
            Twist,
            'rc_car/cmd_vel',
            self.cmd_callback,
            10)

        self.get_logger().info("MotorHAT ROS2 controller started!")


    # def cmd_callback(self, msg):
    #     lin = msg.linear.x * 1.2
    #     ang = -msg.angular.z

    #     self.control_steering(ang)
    #     self.control_motor(lin)



    def cmd_callback(self, msg):
        v = msg.linear.x
        omega = -msg.angular.z

        # 0에 가까운 v로 인한 발산 방지
        v_for_curv = v if abs(v) > MIN_SPEED else (MIN_SPEED if v >= 0 else -MIN_SPEED)

        # yaw rate -> steering angle
        steer_rad = math.atan(WHEEL_BASE * omega / v_for_curv)
        steer_rad = max(min(steer_rad, MAX_STEER), -MAX_STEER)

        # -1~1 정규화해서 기존 servo 함수에 전달
        steer_norm = steer_rad / MAX_STEER

        self.control_steering(steer_norm)
        self.control_motor(v)


    def control_steering(self, ang):
        """
        ang : -1(left) ~ 1(right)
        """
        pwm = int(self.SERVO_CENTER + ang * (self.SERVO_RIGHT - self.SERVO_CENTER))

        # clamp
        pwm = max(min(pwm, self.SERVO_RIGHT), self.SERVO_LEFT)

        self.servo.setPWM(0, 0, pwm)
        self.get_logger().info(f"Steering: {pwm}")




    def control_motor(self, lin):
        """
        lin: m/s (ROS Twist linear.x)
        """
        if abs(lin) < MIN_DEADBAND:
            self.motor.run(Raspi_MotorHAT.RELEASE)
            self.get_logger().info("Motor: STOP")
            return

        # m/s -> PWM
        scale = min(abs(lin) / MAX_LINEAR, 1.0)
        speed = int(scale * self.MOTOR_MAX)

        if lin > 0:
            self.motor.setSpeed(speed)
            self.motor.run(Raspi_MotorHAT.FORWARD)
            self.get_logger().info(f"Motor: FORWARD {speed}")
        else:
            self.motor.setSpeed(speed)
            self.motor.run(Raspi_MotorHAT.BACKWARD)
            self.get_logger().info(f"Motor: BACKWARD {speed}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorHatController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.motor.run(Raspi_MotorHAT.RELEASE)
    node.servo.setPWM(0, 0, node.SERVO_CENTER)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

