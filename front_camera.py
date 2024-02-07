#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2 as cv
import numpy as np
import jetson_inference
import jetson_utils

from robot_vision.variable import FRONT_CAMERA_INDEX, WIDTH, HEIGHT
from robot_vision.variable import MODEL_PATH, LABEL_PATH, THRESHOLD


class FrontCamera(Node):
    def __init__(self):

        # Pengaturan ROS2
        super().__init__(node_name='front_camera')
        self.previous_command = 0
        self.open_camera = self.create_subscription(Int32, '/camera', self.receiver, 10)
        
        # Pengaturan JST
        self.net_ = jetson_inference.detectNet(
            model=MODEL_PATH, 
            labels=LABEL_PATH,
            input_blob='input_0',
            output_cvg='scores',
            output_bbox='boxes', 
            threshold=THRESHOLD
            )
        
        # Pengaturan kamera
        self.front_camera = cv.VideoCapture(FRONT_CAMERA_INDEX)
        self.front_camera.set(3, WIDTH)
        self.front_camera.set(4, HEIGHT)

    def receiver(self, msg:Int32):
        if self.previous_command != msg:
            if self.previous_command == 1:
                self.processing_image()
                self.get_logger().info('Memulai deteksi...')
            else:
                self.close_camera()
                
            self.previous_command = msg
        else:
            self.get_logger().info('Standby...')

    def processing_image(self):
        ret, frame = self.front_camera.read()
        if ret:
            frame_cuda = jetson_utils.cudaFromNumpy(frame)
            detections = self.net_.Detect(frame_cuda)
            for info in detections:
                x1, y1, x2, y2, centeroid = int(info.Left), int(info.Top), int(info.Right), int(info.Bottom), info.Center
                class_name = self.net_.GetClassDesc(info.ClassID)

                if class_name == 'ROBOT':
                    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, (0, 255, 0), -1)
                    cv.putText(frame, class_name, (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)

                elif class_name == 'BOLA':
                    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
                    cv.putText(frame, class_name, (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 1.5, (0, 165, 255), 2)
                    cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, (0, 0, 0), -1)

                elif class_name == 'PENGHALANG':
                    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv.putText(frame, class_name, (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 255), 2)
                    cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, (0, 0, 255), -1)

                elif class_name == 'GAWANG':
                    cv.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv.putText(frame, class_name, (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
                    cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, (255, 0, 0), -1)

        fps_text = "FPS: {:.0f}".format(self.net_.GetNetworkFPS())
        cv.putText(frame, fps_text, (10, 20), cv.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)
        cv.imshow('Result', frame)
        cv.waitKey(1)

    def close_camera(self):
        self.front_camera.release()
        self.get_logger().info('Mematikan kamera...')


def main(args=None):
    rclpy.init(args=args)
    node = FrontCamera()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()