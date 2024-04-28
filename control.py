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
        self.front_message: str = None
        self.omni_message: str = None
        self.setup_serial()

    def front_callback(self, msg: String) -> None:
        '''
        Callback for front camera.
        '''
        self.front_message = str(msg.data)
        self.check_trigger_condition()

    def omni_callback(self, msg: String) -> None:
        '''
        Callback for omni camera.
        '''
        self.omni_message = str(msg.data)
        self.check_trigger_condition()

    def setup_serial(self) -> None:
        '''
        Setup for serial communication to arduino.
        '''
        self.serial_port = serial.Serial(SETTINGS['serialArduino']['comPort'], baudrate=SETTINGS['serialArduino']['baudrate'])

    def serial_arduino(self, msg: str) -> None:
        '''
        Sending message to arduino using serial.
        '''
        self.serial_port.write(msg.encode())
        self.serial_port.write(b'\n')

    def check_trigger_condition(self) -> None:
        default = 'None'

        # Check if both front and omni messages have been received
        if self.front_message is None or self.omni_message is None:
            return

        omni_data = self.omni_message.split(',')
        front_data = self.front_message.split(',')

        # Process omni data
        sumbux = omni_data[0] if omni_data else default
        sumbuy = omni_data[1] if omni_data else default

        classes = front_data[0] if front_data else default
        depth_est = default
        center_x = default
        center_y = default
        goal = default

        if classes == 'ROBOT':
            depth_est = front_data[1] if front_data else default
            center_x = front_data[2] if front_data else default
            center_y = front_data[3] if front_data else default
            goal = front_data[4] if front_data else default
        
        if classes == 'targetGoal':
            goal = front_data[4] if front_data else default

        # Construct data dictionary
        data = {
            'sumbux':sumbux,
            'sumbuy':sumbuy,
            'class':classes,
            'depth':depth_est,
            'centeroid':(center_x, center_y),
            'goal':goal
        }

        # Convert data to JSON
        json_data = json.dumps(data)

        # Log JSON data
        self.get_logger().info(f'{json_data}')

        # Reset messages after processing
        self.front_message = None
        self.omni_message = None

        # Send JSON data to Arduino
        self.serial_arduino(json_data)

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
