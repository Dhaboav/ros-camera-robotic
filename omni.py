#!/usr/bin/env python3
import rclpy
import cv2 as cv
import numpy as np
from typing import Tuple
from rclpy.node import Node
from std_msgs.msg import String
from ros_robot.setupROS import SETTINGS


class Omni(Node):
    def __init__(self) -> None:
        '''
        Initialize the Omni class, which handles image processing and ball detection.
        '''   
        super().__init__(node_name='omni')
        self.omni_pubs_ = self.create_publisher(String, '/camera/omni', 10)
        self.setup_camera()
        self.setup_color()
    
    def send_ball_data(self, message:str):
        msg = String()
        msg.data = message
        self.omni_pubs_.publish(msg)
        self.get_logger().info(f"Published message: {message}")

    # setup start  ===============================================================
    def setup_camera(self) -> None:
        '''
        Setup camera for capturing video and omni center.
        '''
        self.CAMERA = cv.VideoCapture(SETTINGS['omniCamera']['index'], cv.CAP_V4L)
        self.CAMERA.set(3, SETTINGS['widthCamera'])
        self.CAMERA.set(4, SETTINGS['heightCamera'])
        self.timer = self.create_timer(0.1, self.ball_detection)

        # Pengaturan titik tengah omni
        self.x_omni = SETTINGS['omniCamera']['xCenter']
        self.y_omni = SETTINGS['omniCamera']['yCenter']

    def setup_color(self) -> None:
        '''
        Setup for color detection.
        '''
        # Pengaturan warna bola (orange)
        self.lower_value = np.array(SETTINGS['omniCamera']['lowerColor'])
        self.upper_value = np.array(SETTINGS['omniCamera']['upperColor'])

        # Pengaturan morphologi
        self.kernel = SETTINGS['omniCamera']['kernelValue']
        self.dilasi = SETTINGS['omniCamera']['dilasiValue']
        self.erosi = SETTINGS['omniCamera']['erodeValue']
    # setup end  ===============================================================

    # function start ===============================================================
    def exponential_function(self, x):
        '''
        Calculate real distance from ball to robot centeroid.
        '''
        return np.exp(0.0154 * x + 3.0524)
    
    def map_float(self, perimeter, in_min, in_max, out_min, out_max):
        return int((perimeter - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    
    def ball_detection(self):
        '''
        Process the captured image, perform ball detection, and display the results.
        '''
        ret, frame = self.CAMERA.read()
        if ret:
            convert_hsv = cv.cvtColor(src=frame, code=cv.COLOR_BGR2HSV)
            color_mask = cv.inRange(src=convert_hsv, lowerb=self.lower_value, upperb=self.upper_value)
            # kernel = np.ones((self.kernel[0], self.kernel[1]), np.uint8)
            # erode = cv.erode(color_mask, kernel, iterations=self.erosi)
            # dilated = cv.dilate(color_mask, kernel, iterations=self.dilasi)
            contours, _ = cv.findContours(color_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            j = 0
            xyj = 0
            angle = 0
            sumbu_x = 0
            sumbu_y = 0
            for contour in contours:
                area = cv.contourArea(contour)

                # Memfilter kontur berdasarkan luas (sesuaikan sesuai kebutuhan)
                if area > 5:
                    x, y, w, h = cv.boundingRect(contour)
                    ((x_bola, y_bola), radius) = cv.minEnclosingCircle(contour)
                    ball_center = (int(x_bola), int(y_bola))

                    j = np.sqrt((x - self.x_omni) ** 2 + (y - self.y_omni) ** 2)
                    if j >=39:
                        xyj = self.exponential_function(j)
                        angle_j_to_x = np.degrees(np.arctan2(x - self.x_omni, y - self.y_omni))

                        if angle_j_to_x > 0:
                            angles = float(self.map_float(angle_j_to_x, 0, 180, 0, 180))
                            angle = angles
                            
                        elif angle_j_to_x < 0:
                            angles = float(self.map_float(angle_j_to_x, 0, -180, 0, -180))
                            angle = angles
                            
                        sumbu_y = xyj * np.cos(np.radians(angle))
                        sumbu_x = xyj * np.sin(np.radians(angle))
                    elif j<39:
                        xyj = self.map_float(j, 20, 39, 0 , 30.9)
                        angle_j_to_x = np.degrees(np.arctan2(x - self.x_omni, y - self.y_omni))

                        if angle_j_to_x > 0:
                            angles = float(self.map_float(angle_j_to_x, 0, 180, 180, 0))
                            angle = angles
                            
                        elif angle_j_to_x < 0:
                            angles = float(self.map_float(angle_j_to_x, 0, -180, -180, 0))
                            angle = angles
                            
                        sumbu_y = xyj * np.cos(np.radians(angle)) 
                        sumbu_x = xyj * np.sin(np.radians(angle)) 
                    
                    self.draw_object(frame, x, y, w, h, ball_center)
            self.display_frame(frame)
            ros_message = f'{sumbu_x:.0f},{sumbu_y:.0f}'
            self.send_ball_data(ros_message)

    def draw_object(self, frame:np.ndarray, x_bola:int, y_bola:int, w_bola:int, h_bola:int, ball_center: Tuple[int, int]):
        '''
        Draw bounding box and label for the detected ball.
        '''
        cv.rectangle(frame, (x_bola, y_bola), (x_bola + w_bola, y_bola + h_bola), (255, 0, 255), 2)
        cv.circle(frame, ball_center, 2, (0, 255, 0), -1)
        cv.circle(frame, (self.x_omni, self.y_omni), 10, (0,255,0), -1)
        cv.line(frame, (self.x_omni, self.y_omni), (ball_center),(0,0,255), 2) 

    def display_frame(self, frame: np.ndarray) -> None:
        '''
        Display the processed frame.
        '''
        cv.imshow('result', frame)
        key = cv.waitKey(1)
        if key == ord('q'):
            cv.destroyAllWindows()
            self.destroy_node()
            self.get_logger().info('Mematikan kamera...')
    # function end ===============================================================

    # etc start ================================================================
    def destroy_node(self):
        '''
        Release resources and destroy the ROS node.
        '''
        self.CAMERA.release()
        cv.destroyAllWindows()
        super().destroy_node()
    # etc end ================================================================

# Main start ==============================================================
def main(args=None):
    rclpy.init(args=args)
    node = Omni()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# Main end ==============================================================