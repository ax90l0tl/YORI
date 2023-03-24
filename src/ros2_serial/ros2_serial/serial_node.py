import rclpy
from rclpy.node import Node
import serial
import unittest
import time
from custom_msgs.msg import Motor


class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')
        self.serial_obj = []
        self.serial_setup()
        self.get_pubs()
        print('Done!')
        #self.get_subs()

    def serial_setup(self):
        #serial port setup
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.get_logger().info('Opening serial connection at port: '+str(port)+' at baudrate: '+str(baud))
        self.ser = serial.Serial(port, baudrate=baud)
        # while True:
        #     try:
        #         self.ser = serial.Serial(port, baudrate=baud)
        #     #try other ports
        #     except serial.SerialTimeoutException:
        #         self.get_logger().info('Serial connection failed at port: ' + str(port))
        #         for i in range(1, 6):
        #             port = '/dev/ttyACM' + str(i)
        #             self.get_logger().info('Trying again at port: ' + str(port))
        #         self.get_logger().info('Connection failed')
        #         self.get_logger().info('Quitting...')
        #         return -1
        #     else:
        #         self.get_logger().info('Connected to port: ' + str(port))
        #         rclpy.Parameter('port', rclpy.Parameter.Type.STRING, port)
        #         return 0

    def get_subs(self):
        #subscriber setup
        self.ser.write(b'sub\n\r')
        while True:
            if self.ser.in_waiting != 0:
                output = str(self.ser.readline().splitlines()).strip().split(',')
                if output[0] != '' and len(output) == 2:
                    print(output[0])
                    print(output[1])
                    self.get_logger().info('Setting up subscriber to topic: '+output[1])
                    #self.subscription = self.create_subscription(str(output[0]), str(output[1]), self.listener_callback, 10)
                    #self.subscription  # prevent unused variable warning
                    #self.ser.write

    def get_pubs(self):
        while True:
            if self.ser.in_waiting != self.ser.write(b'pub\n\r') and self.ser.in_waiting != 0:
                output = str(self.ser.readline().splitlines()).strip().split(',')
                if output[0] != '' and len(output) == 2:
                    print(output[0])
                    print(output[1])
                    self.get_logger().info('Setting up publisher for topic: '+str(output[1]))
                    break


    def listener_callback(self, msg):
        self.ser.reset_output_buffer()
        val = str(msg.val)
        mode = str(msg.mode)
        output = val+','+mode+'\n\r'
        self.ser.write(bytes(output, 'utf-8'))

    def main(self, node, args):
        while rclpy.ok():
            rclpy.spin_once(self)
    def destroy_node(self) -> bool:
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    while rclpy.ok():
        rclpy.spin_once(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
