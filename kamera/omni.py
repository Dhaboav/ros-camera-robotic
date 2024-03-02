#!/usr/bin/env python3
import json
import rclpy
import serial
import cv2 as cv
import numpy as np
from pathlib import Path
from rclpy.node import Node


class Omni(Node):
    def __init__(self, OCI, width, height, lower, upper,
                x_omni, y_omni, com, kernel, erosi, dilasi):
        
        super().__init__(node_name='omni')

        # Pengaturan kamera
        self.omni_camera = cv.VideoCapture(OCI, cv.CAP_V4L)
        self.omni_camera.set(3, width)
        self.omni_camera.set(4, height)
        
        # Pengaturan warna bola (orange)
        self.lower_value = lower
        self.upper_value = upper

        # Pengaturan titik tengah omni
        self.x_omni = x_omni
        self.y_omni = y_omni

        # Pengaturan morphologi
        self.kernel = kernel
        self.dilasi = dilasi
        self.erosi = erosi

        # Pengaturan ROS2
        self.timer = self.create_timer(0.1, self.ball_detection)

        # Pengaturan serial arduino
        self.serial_port = serial.Serial(com, baudrate=9600)

    # Menghitung distance asli dari bola ke titik pusat robot
    def exponential_function(self, x):
        return np.exp(0.0154 * x + 3.0524)
    
    def map_float(self, perimeter, in_min, in_max, out_min, out_max):
        return int((perimeter - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    
    def ball_detection(self):
        distance_estimation = 0
        ret, frame = self.omni_camera.read()
        if ret:
            convert_hsv = cv.cvtColor(src=frame, code=cv.COLOR_BGR2HSV)
            color_mask = cv.inRange(src=convert_hsv, lowerb=self.lower_value, upperb=self.upper_value)
            kernel = np.ones((self.kernel[0], self.kernel[1]), np.uint8)
            erode = cv.erode(color_mask, kernel, iterations=self.erosi)
            dilated = cv.dilate(erode, None, iterations=self.dilasi)
            contours, _ = cv.findContours(color_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            distance = 0
            for contour in contours:
                area = cv.contourArea(contour)

                # Memfilter kontur berdasarkan luas (sesuaikan sesuai kebutuhan)
                if area > 1:
                    x, y, w, h = cv.boundingRect(contour)
                    # Menggambar bounding box pada frame
                    cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)

                    ((x, y), radius) = cv.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    distance = np.sqrt((x - self.x_omni) ** 2 + (y - self.y_omni) ** 2)  # Menghitung distance piksel dari objek ke titik tengah
                    distance_estimation = self.exponential_function(distance)

                    # Titik tengah
                    cv.circle(frame, center, 2, (0, 255, 0), -1)
                    cv.putText(img=frame, text=f'{distance_estimation:.0f} CM', org=(10,20), fontFace=cv.FONT_HERSHEY_PLAIN,
                               fontScale=1.0, color=(0, 255, 0), thickness=1)
                    
                    cv.circle(img=frame, center=(self.x_omni, self.y_omni), radius=10,
                              color=(0,255,0), thickness= -1)
                    
                    cv.line(img=frame, pt1=(self.x_omni, self.y_omni), pt2=(int(x), int(y)),
                            color=(0,0,255), thickness=2)
                   
            cv.imshow('dark', dilated)
            cv.imshow('result', frame)
            key = cv.waitKey(1)
            if key == ord('q'):
                cv.destroyAllWindows()
                self.destroy_node()
            
            self.get_logger().info(f'{distance_estimation:.0f}')
            self.serial_arduino(int(distance_estimation))

    def serial_arduino(self, msg:int):
        self.serial_port.write(bytes([msg]))
        self.serial_port.flush()

    def destroy_node(self):
        self.omni_camera.release()
        cv.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    current_folder = Path(__file__).resolve().parent
    parent_folder = current_folder.parent
    setting_path = parent_folder / 'settings.json'

    # Read JSON data from a file
    with open(setting_path, "r") as json_file:
        data = json.load(json_file)

    rclpy.init(args=args)
    node = Omni(OCI=data["camera"]["OCI"], width=data["camera"]["WIDTH"], 
                height=data["camera"]["HEIGHT"], 
                lower=data["omniCamera"]["LOWER"], upper=data["omniCamera"]["UPPER"], 
                x_omni=data["omniCamera"]["X"], y_omni=data["omniCamera"]["Y"], 
                com=data["omniCamera"]["COM"], kernel=data["omniCamera"]["KERNEL"], 
                erosi=data["omniCamera"]["ERODE"], dilasi=data["omniCamera"]["DILATE"]
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()