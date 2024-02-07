#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2 as cv
import serial

from robot_vision.variable import OMNI_CAMERA_INDEX, WIDTH, HEIGHT
from robot_vision import LOWER, UPPER, COM


class OmniCamera(Node):
    def __init__(self):
        super().__init__(node_name='omni_camera')

        # Pengaturan kamera
        self.front_camera = cv.VideoCapture(OMNI_CAMERA_INDEX)
        self.front_camera.set(3, WIDTH)
        self.front_camera.set(4, HEIGHT)
        
        # Pengaturan warna bola (orange)
        self.lower_value = LOWER
        self.upper_value = UPPER

        # Pengaturan ROS2
        self.camera_pubs_ = self.create_publisher(Int32, '/camera', 10)
        self.create_timer(1.0, self.serial_arduino)

        # Pengaturan arduino
        self.serial_port_ = serial.Serial(COM, baudrate=9600)
        self.session_ = 0

    def send_command(self):
        msg = Int32()
        msg.data = self.session_
        self.camera_pubs_.publish(msg)
        self.get_logger().info(str(msg.data))

    def serial_arduino(self):
        # read data from arduino
        message = self.serial_port_.readline()
        message = message.decode('utf-8').strip()
        self.session_ = int(message)

    def ball_detection(self):
        ret, frame = self.front_camera.read()
        if ret:
            convert_hsv = cv.cvtColor(src=frame, code=cv.COLOR_BGR2HSV)
            color_mask = cv.inRange(src=convert_hsv, lowerb=self.lower_value, upperb=self.upper_value)
            result = cv.bitwise_and(src1=frame, src2=frame, mask=color_mask)
            cv.imshow('omni', result)
            cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = OmniCamera()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()