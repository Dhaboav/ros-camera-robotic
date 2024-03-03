#!/usr/bin/env python3
import json
import rclpy
import serial
import threading
from pathlib import Path
from rclpy.node import Node
from ros_robot.komunikasi.client_side import Client


class RosReceiverControl(Node):
    def __init__(self, com:str):
        super().__init__('Testing')
        self.__serial_port = serial.Serial(com, baudrate=9600)
        self.__previous_msg = 'k'
        self.__tester = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(self.__previous_msg)

    def receiver_callback(self, msg:str):
        if self.__previous_msg != msg:
            self.__previous_msg = msg
            self.serial_arduino(msg)
        else:
            pass
            
    def serial_arduino(self, msg:str):
        self.__serial_port.write(msg.encode())
        

def main(args=None):
    current_folder = Path(__file__).resolve().parent
    parent_folder = current_folder.parent
    setting_path = parent_folder / 'settings.json'

    # Read JSON data from a file
    with open(setting_path, "r") as json_file:
        data = json.load(json_file)

    client = Client(host=data["control"]["IP_BASESTATION"])
    connect_thread = threading.Thread(target=client.connect)
    connect_thread.start()
    
    # Ros
    rclpy.init(args=args)
    node = RosReceiverControl(com=data["control"]["COM"])
    client.set_message_callback(callback=node.receiver_callback)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
