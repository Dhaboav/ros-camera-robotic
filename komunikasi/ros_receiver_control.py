#!/usr/bin/env python3
import json
import rclpy
# import serial
import threading
from pathlib import Path
from rclpy.node import Node
from ros_robot.komunikasi.client_side import Client



class RosReceiverControl(Node):
    def __init__(self):
        super().__init__('Testing')
        # self.__serial_port = serial.Serial(COM, baudrate=9600)
        self.__previous_msg = 'S'
        self.__tester = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(self.__previous_msg)

    def receiver_callback(self, msg:str):
        if self.__previous_msg != msg:
            self.__previous_msg = msg
         

    # def serial_arduino(self, msg:Int32):
    #     self.serial_port.write(bytes([msg.data]))
    #     self.serial_port.flush()


def main(args=None):
    current = Path(__file__).resolve().parent
    parent = current.parent
    setings = parent / 'settings.json'

    # Read JSON data from a file
    with open(setings, "r") as json_file:
        data = json.load(json_file)

    client = Client(host=data["IP_BASESTATION"])
    connect_thread = threading.Thread(target=client.connect)
    connect_thread.start()
    
    # Ros
    rclpy.init(args=args)
    node = RosReceiverControl()
    client.set_message_callback(callback=node.receiver_callback)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()