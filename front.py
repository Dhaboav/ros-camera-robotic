#!/usr/bin/env python3
import rclpy
import serial
import cv2 as cv
import numpy as np
import jetson_utils
from ros_robot.model import Model
import jetson_inference
from rclpy.node import Node


# Bagian kelas mulai ==================================================================
class Front(Node):
    def __init__(self) -> None:

        # Pengaturan umum
        model = Model()
        config: list = model.front_camera
        self.depth: list[int] = model.depth
        self.focal: int = model.focal_length

        # Pengaturan ROS2
        super().__init__(node_name='front')
        
        # Pengaturan Jetson
        self.net_ = jetson_inference.detectNet(
            model=config[4], 
            labels=config[5],
            input_blob='input_0',
            output_cvg='scores',
            output_bbox='boxes', 
            threshold=config[6]
        )
        
        # Pengaturan kamera
        self.front_camera = cv.VideoCapture(config[2], cv.CAP_V4L)
        self.front_camera.set(3, config[0])
        self.front_camera.set(4, config[1])
        self.front_camera.set(cv.CAP_PROP_AUTOFOCUS, 0)
        self.timer = self.create_timer(0.1, self.processing_image)

        # Pengaturan serial arduino
        self.serial_port = serial.Serial(config[3], baudrate=9600)

    def focal_length(self, pixel_width, distance:int, real_width:int) -> float:
        return (pixel_width * distance) / real_width
    
    def depth_estimation(self, focal_length:int, real_width:int, pixel_width) -> float:
        return (real_width * focal_length) / pixel_width
    
    def processing_image(self) -> None:
        ret, frame = self.front_camera.read()
        if ret:
            frame_cuda = jetson_utils.cudaFromNumpy(frame)
            detections = self.net_.Detect(frame_cuda)

            # Pengaturan data kelas
            class_properties = {
                'ROBOT': {'color': (0, 255, 0), 'text_color': (0, 255, 0)},
                'BOLA': {'color': (0, 165, 255), 'text_color': (0, 165, 255)},
                'PENGHALANG': {'color': (0, 0, 255), 'text_color': (0, 0, 255)},
                'GAWANG': {'color': (255, 0, 0), 'text_color': (255, 0, 0)}
            }

            y1g, x1p, x2p, y2p = 0, 0, 0, 0
            center_gawang = (0, 0)
            center_penghalang = (0, 0)
            for info in detections:
                x1, y1, x2, y2, centeroid = int(info.Left), int(info.Top), int(info.Right), int(info.Bottom), info.Center
                class_name = self.net_.GetClassDesc(info.ClassID)

                # Mengambil properti warna dan teks dari dictionary
                class_props = class_properties.get(class_name)
                if class_props:
                    color = class_props['color']
                    text_color = class_props['text_color']
                    pixel_width = int((x2 - x1) * frame.shape[1]) if 0 <= x1 <= 1 and 0 <= x2 <= 1 else int(x2 - x1)
                    depth_est = self.depth_estimation(self.focal, self.depth.get(class_name), pixel_width)

                    # Draw
                    cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv.putText(frame, f'{class_name}:{int(depth_est)} CM', (x1, y1 - 10), cv.FONT_HERSHEY_PLAIN, 1.5, text_color, 2)
                    cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, color, -1)

                if class_name == 'GAWANG':
                    center_gawang = centeroid
                    y1g = y1
                if class_name == 'PENGHALANG':
                    center_penghalang = centeroid
                    x1p, x2p, y2p = x1, x2, y2

                #  Kiri
                x_kiri = x1p - center_gawang[0] 
                y_kiri = y2p - center_gawang[1]
                #  Kanan
                x_kanan = x2p + center_penghalang[0]
                y_kanan = y2p - center_penghalang[1]

                x_target, y_target = (int(x_kiri), int(y_kiri)) if center_gawang[0] > center_penghalang[1] else (int(x_kanan), int(y_kanan))
                if center_gawang[1] > y1g and center_penghalang[1] > y1g:
                    cv.circle(frame, (x_target, y_target), 5, (255, 255, 255), -1)

        fps_text = 'FPS: {:.0f}'.format(self.net_.GetNetworkFPS())
        cv.putText(frame, fps_text, (10, 20), cv.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), 2)
        cv.imshow('Result', frame)
        key = cv.waitKey(1)
        if key == ord('q'):
            cv.destroyAllWindows()
            self.destroy_node()
            self.get_logger().info('Mematikan kamera...')

    def destroy_node(self) -> None:
        self.front_camera.release()
        cv.destroyAllWindows()
        super().destroy_node()
    
    def serial_arduino(self, msg:str) -> None:
        self.serial_port.write(msg.encode())
        self.serial_port.flush()
# Bagian kelas selesai ===========================================================

# Bagian main mulai ==============================================================
def main(args=None):
    rclpy.init(args=args)
    node = Front()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# Bagian main selesai ============================================================