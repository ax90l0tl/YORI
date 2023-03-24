import sys

from custom_msg_srv.srv import Funnel
import rclpy
from rclpy.node import Node


class FunnelClient(Node):

    def __init__(self):
        super().__init__('funnel_client')
        self.cli = self.create_client(Funnel, 'funnel')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Funnel.Request()

    def send_request(self, a, b):
        self.req.action = "UP"
        self.amount = 32
        self.req.time = 100
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    funnel_client = FunnelClient()
    response = funnel_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    funnel_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    funnel_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()