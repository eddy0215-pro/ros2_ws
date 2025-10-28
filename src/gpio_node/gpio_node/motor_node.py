#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# ==================== 핀 설정 ====================
Motor_A_EN    = 4
Motor_B_EN    = 17
Motor_A_Pin1  = 26
Motor_A_Pin2  = 21
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 1
Dir_backward  = 0

left_forward  = 1
left_backward = 0
right_forward = 0
right_backward= 1

pwm_A = None
pwm_B = None

# ==================== GPIO 및 모터 함수 ====================
def motorStop():
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_B_Pin1, GPIO.LOW)
    GPIO.output(Motor_B_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    GPIO.output(Motor_B_EN, GPIO.LOW)

def setup():
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)

    motorStop()
    pwm_A = GPIO.PWM(Motor_A_EN, 1000)
    pwm_B = GPIO.PWM(Motor_B_EN, 1000)

def motor_left(status, direction, speed):
    if status == 0:
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)
    else:
        if direction == Dir_forward:
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
        else:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
        pwm_B.start(0)
        pwm_B.ChangeDutyCycle(speed)

def motor_right(status, direction, speed):
    if status == 0:
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
    else:
        if direction == Dir_forward:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
        else:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
        pwm_A.start(0)
        pwm_A.ChangeDutyCycle(speed)

def destroy():
    motorStop()
    pwm_A.stop()
    pwm_B.stop()
    GPIO.cleanup()

def map_value(x, in_min, in_max, out_min, out_max):
    """입력값을 주어진 범위에 맞게 선형 매핑"""
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def move_from_twist(linear_x, angular_z, wheel_base=1.7, 
                    max_speed=100, max_linear=1.0, max_angular=1.0,
                    min_pwm=50):
    """
    ROS2 Twist 메시지 기반 차동구동 모터 제어
    linear_x: m/s 단위 전진 속도
    angular_z: rad/s 단위 회전 속도
    wheel_base: 바퀴 간 거리 (m)
    min_pwm: 모터가 움직이기 시작하는 최소 PWM (기본 50)
    """

    # -----------------------------
    # 1️⃣ 속도 → PWM 변환 (map 함수)
    # -----------------------------
    v = map_value(linear_x, -max_linear, max_linear, -max_speed, max_speed)
    w = map_value(angular_z, -max_angular, max_angular, -max_speed, max_speed)

    # -----------------------------
    # 2️⃣ 차동구동 계산
    # -----------------------------
    left_speed = v - (w * wheel_base / 2.0)
    right_speed = v + (w * wheel_base / 2.0)

    # -----------------------------
    # 3️⃣ 곡선 회전 감속 보정
    # -----------------------------
    if abs(linear_x) > 0.1 and abs(angular_z) > 0.1:
        curve_factor = 0.8
    else:
        curve_factor = 1.0

    left_speed *= curve_factor
    right_speed *= curve_factor

    # -----------------------------
    # 4️⃣ 데드존 보정
    # -----------------------------
    def apply_deadzone(pwm_val):
        if pwm_val == 0:
            return 0
        mapped = map_value(abs(pwm_val), 0, max_speed, min_pwm, max_speed)
        return int(max(min(mapped, max_speed), 0))

    left_pwm = apply_deadzone(left_speed)
    right_pwm = apply_deadzone(right_speed)

    # -----------------------------
    # 5️⃣ 방향 결정
    # -----------------------------
    left_dir = Dir_forward if left_speed >= 0 else Dir_backward
    right_dir = Dir_forward if right_speed >= 0 else Dir_backward

    # -----------------------------
    # 6️⃣ 정지 조건
    # -----------------------------
    if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
        motorStop()
        return

    # -----------------------------
    # 7️⃣ 모터 구동
    # -----------------------------
    motor_left(1, left_dir, left_pwm)
    motor_right(1, right_dir, right_pwm)

    # -----------------------------
    # 디버깅용 출력
    # -----------------------------
    dir_map = {Dir_forward: "FORWARD", Dir_backward: "BACKWARD"}

    print(f"[Motor] LEFT : {dir_map[left_dir]:<7} PWM={left_pwm:>3} | "
        f"RIGHT : {dir_map[right_dir]:<7} PWM={right_pwm:>3} | "
        f"linear_x={linear_x:.2f}, angular_z={angular_z:.2f}")

# ==================== ROS2 노드 ====================
class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        setup()
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("✅ Motor Node started! Subscribing to /cmd_vel ...")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        move_from_twist(linear_x, angular_z)

    def destroy_node(self):
        destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()