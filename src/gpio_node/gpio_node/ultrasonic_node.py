#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import RPi.GPIO as GPIO
import time

# ==================== 초음파 센서 핀 ====================
Tr = 11
Ec = 8

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # GPIO 초기화 (한 번만)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Ec, GPIO.IN)

        # Trigger 서비스 등록
        self.srv = self.create_service(Trigger, 'ultrasonic/measure', self.handle_measure)
        self.get_logger().info("Ultrasonic Service ready! Call 'ultrasonic/measure' to get distance.")

    def check_distance(self, timeout=0.02):
        """센서 1회 측정"""
        GPIO.output(Tr, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(Tr, GPIO.LOW)

        start_time = time.time()
        # Echo 상승 대기
        while not GPIO.input(Ec):
            if time.time() - start_time > timeout:
                raise TimeoutError("Echo signal not received")
        t1 = time.time()

        # Echo 하강 대기
        while GPIO.input(Ec):
            if time.time() - t1 > timeout:
                raise TimeoutError("Echo signal stuck HIGH")
        t2 = time.time()

        distance_m = (t2 - t1) * 340 / 2
        return distance_m

    def handle_measure(self, request, response):
        """서비스 요청이 들어올 때만 측정"""
        try:
            distance = self.check_distance()
            distance_cm = distance * 100
            response.success = True
            response.message = f"{distance_cm:.1f} cm"
            # 로그 출력 최소화
            self.get_logger().info(f"Measured distance: {distance_cm:.1f} cm")
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            self.get_logger().error(f"Distance measurement failed: {e}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)  # 요청 들어올 때만 handle_measure 실행
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
