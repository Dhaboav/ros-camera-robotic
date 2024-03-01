#!/usr/bin/env python3
import json
import rclpy
import serial
import cv2 as cv
import jetson_utils
import jetson_inference
from pathlib import Path
from rclpy.node import Node


class Front(Node):
    def __init__(self, model, label, threshold,
                FCI, width, height, com):

        # Pengaturan ROS2
        super().__init__(node_name='front')
        
        # Pengaturan JST
        self.net_ = jetson_inference.detectNet(
            model=model, 
            labels=label,
            input_blob='input_0',
            output_cvg='scores',
            output_bbox='boxes', 
            threshold=threshold
            )
        
        # Pengaturan kamera
        self.front_camera = cv.VideoCapture(FCI, cv.CAP_V4L)
        self.front_camera.set(3, width)
        self.front_camera.set(4, height)
        self.timer = self.create_timer(0.1, self.processing_image)

        # Pengaturan serial arduino
        self.serial_port = serial.Serial(com, baudrate=9600)


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

        self.serial_arduino(class_name)
        fps_text = "FPS: {:.0f}".format(self.net_.GetNetworkFPS())
        cv.putText(frame, fps_text, (10, 20), cv.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), 2)
        cv.imshow('Result', frame)
        key = cv.waitKey(1)
        if key == ord('q'):
            cv.destroyAllWindows()
            self.destroy_node()
            self.get_logger().info('Mematikan kamera...')

    def destroy_node(self):
        self.front_camera.release()
        cv.destroyAllWindows()
        super().destroy_node()
    
    def serial_arduino(self, msg:str):
        self.serial_port.write(msg.encode())
        self.serial_port.flush()

def main(args=None):
    current_folder = Path(__file__).resolve().parent
    parent_folder = current_folder.parent
    setting_path = parent_folder / 'settings.json'

    # Read JSON data from a file
    with open(setting_path, "r") as json_file:
        data = json.load(json_file)


    rclpy.init(args=args)
    node = Front(model=data["MODEL"], label=data["LABEL"], threshold=data["THRESHOLD"], 
                FCI=data["FCI"], width=data["WIDTH"], height=data["HEIGHT"], com=data["FRONT_COM"])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()