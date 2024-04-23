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
        self.serial_port = serial.Serial(SETTINGS['serialArduino']['comPort'], baudrate=SETTINGS['serialArduino']['baudrate'])
    
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
        default_value = 0
        default = 'None'
        sumbux = default
        sumbuy = default
        objek = default
        jarak= default
        x_center= default
        y_center= default
        anglegol= default

        # Ensure omni_message is not None
        if self.omni_message is not None:
            omni_data = self.omni_message.split(',')
            sumbux = omni_data[1]
            sumbuy = omni_data[2]

        # Check the type of message based on the number of parts
        if self.front_message is not None:
            # Split the message
            message_parts = self.front_message.split(',')

            # Check the type of message based on the number of parts
            if len(message_parts) == 1:
                # Angle message format: f'{anglegol:.2f}'
                anglegol = float(message_parts[0])
                objek = default
                jarak = default_value
                x_center = default_value
                y_center = default_value
            elif len(message_parts) == 4:
                # Class data message format: f'{class_name},{depth_est},{int(centeroid[0])},{int(centeroid[1])}'
                objek = message_parts[0]
                jarak = message_parts[1]
                x_center = message_parts[2]
                y_center = message_parts[3]
                anglegol = default

        # Construct data dictionary
        data = {
            'sumbux': sumbux,
            'sumbuy': sumbuy,
            'objek': objek,
            'jarak': jarak,
            'x_center': x_center,
            'y_center': y_center,
            'anglegol': anglegol
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