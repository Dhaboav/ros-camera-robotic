#!/usr/bin/env python3
import rclpy
import serial
import cv2 as cv
import numpy as np
import jetson_utils
import jetson_inference
from rclpy.node import Node
from typing import Dict, Tuple
from ros_robot.setupROS import SETTINGS


class Front(Node):
    def __init__(self) -> None:
        '''
        Initialize the Front class, which handles image processing and object detection.
        '''
        super().__init__(node_name='front')
        self.setup_jetson()
        self.setup_camera()
        self.setup_distance()

    # setup start  ===============================================================
    def setup_jetson(self) -> None:
        '''
        Setup Jetson Inference for object detection.
        '''
        self.net_ = jetson_inference.detectNet(
            model=SETTINGS['frontCamera']['modelPath'], 
            labels=SETTINGS['frontCamera']['labelPath'],
            input_blob='input_0',
            output_cvg='scores',
            output_bbox='boxes', 
            threshold=SETTINGS['frontCamera']['threshold']
        )

    def setup_camera(self) -> None:
        '''
        Setup camera for capturing video.
        '''
        self.CAMERA = cv.VideoCapture(SETTINGS['frontCamera']['index'], cv.CAP_V4L)
        self.CAMERA.set(3, SETTINGS['widthCamera'])
        self.CAMERA.set(4, SETTINGS['heightCamera'])
        self.CAMERA.set(cv.CAP_PROP_AUTOFOCUS, 0)
        self.timer = self.create_timer(0.1, self.processing_image)

    def setup_distance(self) -> None:
        '''
        Setup for distance estimation variabel.
        '''
        self.focal = SETTINGS['focalLength']
        self.real_dist = SETTINGS['realDist']
        self.depth = SETTINGS['depthData']
    
    def setup_serial(self) -> None:
        '''
        Setup for serial communication to arduino.
        '''
        # self.serial_port = serial.Serial(config[3], baudrate=9600)
    # setup end  ==================================================================

    # function start  =============================================================
    def processing_image(self) -> None:
        '''
        Process the captured image, perform object detection, and display the results.
        '''
        ret, frame = self.CAMERA.read()
        if ret:
            frame_cuda = jetson_utils.cudaFromNumpy(frame)
            detections = self.net_.Detect(frame_cuda)
            for info in detections:
                x1, y1, x2, y2, centeroid = int(info.Left), int(info.Top), int(info.Right), int(info.Bottom), info.Center
                class_name = self.net_.GetClassDesc(info.ClassID)
                class_props = self.get_class_properties(class_name)
                if class_props:
                    self.draw_object(frame, class_name, x1, y1, x2, y2, centeroid, class_props)
            self.display_frame(frame)

    def get_class_properties(self, class_name: str) -> Dict[str, Tuple[int, int, int]]:
        '''
        Get properties (color, text color) of the detected class.
        '''
        class_properties = {
            'ROBOT': {'color': (0, 255, 0), 'text_color': (0, 255, 0)},
            'BOLA': {'color': (0, 165, 255), 'text_color': (0, 165, 255)},
            'PENGHALANG': {'color': (0, 0, 255), 'text_color': (0, 0, 255)},
            'GAWANG': {'color': (255, 0, 0), 'text_color': (255, 0, 0)}
        } 
        return class_properties.get(class_name, {})
    
    def draw_object(self, frame: np.ndarray, class_name:str, x1: int, y1: int, 
                    x2: int, y2: int, centeroid: Tuple[int, int], 
                    class_props: Dict[str, Tuple[int, int, int]]) -> None:
        '''
        Draw bounding box and label for the detected object.
        '''
        color = class_props['color']
        text_color = class_props['text_color']
        pixel_width = int(x2 - x1)
        depth_est = self.depth_estimation(pixel_width, class_name)
        # angle = np.interp(centeroid[0], [0, 640], [-22.5, 22.5])
        # sumbu_x = depth_est * np.sin(angle)
        # sumbu_y = depth_est * np.cos(angle)
        cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv.putText(frame, f'{class_name}: {depth_est} CM', (x1, y1), cv.FONT_HERSHEY_PLAIN, 1.5, text_color, 2)
        cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, color, -1)

        # anglegol = 0
        # x1g,y1g, x2g, y2g, x1p,y1p, x2p,y2p, x_gol, x_kiri,x_kanan, y_gol = 0, 0, 0, 0, 0, 0,0,0,0,0,0,0
        # if class_name == 'PENGHALANG':
        #     center_penghalang = centeroid
        #     jarak_penghalang = depth_est
        #     x1p,y1p, x2p, y2p = x1, y1, x2, y2
        # if class_name == 'GAWANG':
        #     center_gawang = centeroid
        #     jarak_gawang = depth_est
        #     x1g,y1g, x2g, y2g = x1, y1, x2, y2

        # #  Kiri
        # # x_kiri = abs((int(x1p) - int(x1g))/2)
        # x_kiri = x1p-x1g
        # y_gol = abs((y2g - y1g)/2)+y1g
        # #  Kanan
        # x_kanan = abs(int((x2g) - int(x2p))/2)
        # if x_kiri > x_kanan:
        #     x_gol = x_kiri
        # elif x_kanan > x_kiri:
        #     x_gol = x_kanan
        # if x_gol > y1g:
        #     cv.circle(frame, (int(x_gol), int(y_gol)), 5, (255, 0, 255), -1)
        #     anglegol = np.interp(x_gol, [0, 640], [-22.5,22.5])
        #     cv.putText(frame, f'{x1g,x2g,x1p,x2p}',(int(x_gol),int(y_gol+50)), cv.FONT_HERSHEY_PLAIN, 1.0, (255,0,255),2)

    def focal_length(self, pixel_width:int, class_name:str) -> int:
        '''
        Estimate the focal length of camera. Only used to get focal length.
        '''
        real_width = self.depth.get(class_name, 0)
        return (pixel_width * self.real_dist) // real_width
    
    def depth_estimation(self, pixel_width: int, class_name:str) -> int:
        '''
        Estimate the depth of the detected object.
        '''
        real_width = self.depth.get(class_name, 0)
        return (real_width * self.focal) // pixel_width

    def display_frame(self, frame: np.ndarray) -> None:
        '''
        Display the processed frame.
        '''
        fps_text = 'FPS: {:.0f}'.format(self.net_.GetNetworkFPS())
        cv.putText(frame, fps_text, (10, 20), cv.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), 2)
        cv.imshow('Result', frame)
        key = cv.waitKey(1)
        if key == ord('q'):
            cv.destroyAllWindows()
            self.destroy_node()
            self.get_logger().info('Mematikan kamera...')
    # function end ================================================================

    # etc start ================================================================
    def destroy_node(self) -> None:
        '''
        Release resources and destroy the ROS node.
        '''
        self.CAMERA.release()
        cv.destroyAllWindows()
        super().destroy_node()
    
    def serial_arduino(self, msg:str) -> None:
        '''
        Sending message to arduino using serial.
        '''
        self.serial_port.write(msg.encode())
        self.serial_port.flush()
    # etc end =================================================================

# Main start ==========================================================
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
# Main end ============================================================