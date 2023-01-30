import rclpy
from rclpy.node import Node
import serial
import time
from custom_msgs.msg import Motor


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        #serial port setup
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.get_logger().info(
            'Opening serial connection at port: '+str(port)+' at baudrate: '+str(baud))
        self.ser = serial.Serial(port, baudrate=baud)

        #subscriber setup
        topic = 'motor_ctrl'
        # topic = ''
        # while topic == '':
        #     topic = self.ser.readline()
        #self.ser.write(b'hi\n\r')
        self.get_logger().info('Setting up subscriber to topic: '+str(topic))
        self.subscription = self.create_subscription(Motor, topic, self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.ser.reset_output_buffer()
        val = str(msg.val)
        mode = str(msg.mode)
        output = val+','+mode+'\n\r'
        self.ser.write(bytes(output, 'utf-8'))

    def destroy_node(self) -> bool:
        self.ser.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    serial_node = SerialNode()
    rclpy.spin(serial_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
