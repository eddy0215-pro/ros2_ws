#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ==================== 초음파 센서 핀 ====================
TRIG_PIN = 11
ECHO_PIN = 8

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # GPIO 초기화
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(ECHO_PIN, GPIO.IN)

        # QoS 설정: 센서용 안정적인 전달
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 퍼블리셔
        self.publisher = self.create_publisher(Range, '/ultrasonic/front', qos_profile)

        # 센서 정보
        self.field_of_view = 0.1
        self.min_range = 0.02
        self.max_range = 4.0

        # 0.1초 타이머 (10Hz)
        self.timer = self.create_timer(1/3, self.publish_distance)
        self.get_logger().info("Ultrasonic Node started, publishing on /ultrasonic/front")

    def measure_distance(self, timeout=0.02):
        """초음파 1회 측정"""
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(TRIG_PIN, GPIO.LOW)

        start_time = time.time()
        while not GPIO.input(ECHO_PIN):
            if time.time() - start_time > timeout:
                raise TimeoutError("Echo not received")
        t1 = time.time()

        while GPIO.input(ECHO_PIN):
            if time.time() - t1 > timeout:
                raise TimeoutError("Echo stuck HIGH")
        t2 = time.time()

        distance_m = (t2 - t1) * 340 / 2
        return distance_m

    def publish_distance(self):
        try:
            distance = self.measure_distance()
            # 범위 제한
            distance = max(self.min_range, min(distance, self.max_range))

            # 새 메시지 생성
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "ultrasonic_front"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.field_of_view
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            msg.range = distance
  
            self.publisher.publish(msg)
            self.get_logger().info(f"Distance: {distance*100:.1f} cm")
        except Exception as e:
            self.get_logger().error(f"Distance measurement failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
