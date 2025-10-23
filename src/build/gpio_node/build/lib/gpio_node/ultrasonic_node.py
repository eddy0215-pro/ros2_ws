#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import RPi.GPIO as GPIO
import time

# ==================== 초음파 센서 핀 ====================
Tr = 11
Ec = 8

def checkdist():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(Ec, GPIO.IN)
    
    GPIO.output(Tr, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(Tr, GPIO.LOW)
    
    while not GPIO.input(Ec):
        pass
    t1 = time.time()
    while GPIO.input(Ec):
        pass
    t2 = time.time()
    
    distance = (t2 - t1) * 340 / 2  # 단위: m
    return distance

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        GPIO.setwarnings(False)
        self.srv = self.create_service(Trigger, 'ultrasonic/measure', self.handle_measure)
        self.get_logger().info("Ultrasonic Service ready! Call 'ultrasonic/measure' to get distance.")

    def handle_measure(self, request, response):
        try:
            distance = checkdist()
            response.success = True
            response.message = f"{distance:.2f} m"
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
        return response

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

if __name__ == '__main__':
    main()
