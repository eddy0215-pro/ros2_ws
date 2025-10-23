#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

# ==================== 기존 move.py 기반 코드 ====================
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
            pwm_B.start(0)
            pwm_B.ChangeDutyCycle(speed)
        else:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)

def motor_right(status, direction, speed):
    if status == 0:
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
    else:
        if direction != Dir_forward:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        else:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(0)
            pwm_A.ChangeDutyCycle(speed)

def move(speed, direction, turn, radius=0.6):
    speed = abs(speed)
    if direction == 'forward':
        left_dir = left_forward
        right_dir = right_forward
    elif direction == 'backward':
        left_dir = left_backward
        right_dir = right_backward
    else:
        motorStop()
        return

    if turn == 'left':
        motor_left(1, left_forward, speed)
        motor_right(1, right_backward, speed)
    elif turn == 'right':
        motor_left(1, left_backward, speed)
        motor_right(1, right_forward, speed)
    else:
        motor_left(1, left_dir, speed)
        motor_right(1, right_dir, speed)

def destroy():
    motorStop()
    pwm_A.stop()
    pwm_B.stop()
    GPIO.cleanup()

# ==================== ROS2 노드 부분 ====================
class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        setup()
        self.subscription = self.create_subscription(
            String,
            'motor/cmd',
            self.listener_callback,
            10
        )
        self.get_logger().info("Motor Node started and waiting for commands...")

    def listener_callback(self, msg):
        """
        메시지 예시: 'forward,none,60'  → 방향, 회전, 속도
        """
        try:
            direction, turn, speed = msg.data.split(',')
            speed = int(speed)
            self.get_logger().info(f"Moving {direction}, turn={turn}, speed={speed}")
            move(speed, direction, turn)
        except Exception as e:
            self.get_logger().error(f"Invalid command: {msg.data}, error={e}")

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
