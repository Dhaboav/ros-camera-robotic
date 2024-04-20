import rclpy
import serial
import json
from rclpy.node import Node
from std_msgs.msg import String
from ros_robot.setupROS import SETTINGS


class Control(Node):
    def __init__(self):
        '''
        Initialize the Control class, which handles logic for switch camera and serial communication.
        '''
        super().__init__('Control')
        self.front_subs_ = self.create_subscription(String, 'camera/front', self.front_callback, 10)
        self.omni_subs_ = self.create_subscription(String, 'camera/omni', self.omni_callback, 10)
        self.front_message:str = None
        self.omni_message:str = None
        self.setup_serial()

    def front_callback(self, msg: String) -> None:
        '''
        Callback for front camera.
        '''
        self.front_message = str(msg.data)
        self.check_trigger_condition()

    def omni_callback(self, msg:String) -> None:
        '''
        Callback for omni camera.
        '''
        self.omni_message = str(msg.data)
        self.check_trigger_condition()

    def setup_serial(self) -> None:
        '''
        Setup for serial communication to arduino.
        '''
        self.serial_port = serial.Serial(SETTINGS['comPort'], baudrate=9600)
    
    def serial_arduino(self, msg:str) -> None:
        '''
        Sending message to arduino using serial.
        '''
        self.serial_port.write(msg.encode())
        self.serial_port.write(b'\n')
    
    def check_trigger_condition(self) -> None:
        '''
        Check if trigger condition is met based on messages from both cameras.
        '''
        if self.front_message is not None and self.omni_message is not None:
            omni = self.omni_message.split(',')
            front = self.front_message.split(',')
            
            if omni[0] == 'BOLA':
                data = {
                    'sumbux':omni[1],
                    'sumbuy':omni[2],
                    'objek':front[0],
                    'jarak':front[1],
                    'x_center':front[2],
                    'y_center':front[3]
                }
                json_data = json.dumps(data)
                self.serial_arduino(json_data)
                self.get_logger().info(f'{json_data}')

            # Reset messages after processing
            self.front_message = None
            self.omni_message = None

# Main start ==========================================================
def main(args=None):
    rclpy.init(args=args)
    node = Control()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# Main end ============================================================  