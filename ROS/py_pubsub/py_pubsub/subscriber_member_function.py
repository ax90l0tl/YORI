import rclpy
from rclpy.node import Node
import serial
from custom_msgs.msg import Motor
#from std_msgs.msg import String
ser = serial.Serial('/dev/ttyACM0')
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Motor,
            'Motor_ctrl',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        ser.reset_output_buffer()
        # ser.write(bytes(str(msg.val) + ', ' + msg.mode + '\n', 'utf-8'))
        ser.write(b"1, coast")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
