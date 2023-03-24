import rclpy
from rclpy.node import Node
from yori_funnel.functions import yori_funnel_functions as func
from custom_msg_srv.msg import Funnel
import numpy as np


class FunnelSubscriber(Node):

    def __init__(self):
        super().__init__('yori_funnel')
        self.funnel = func.yori_funnel()
        self.subscription = self.create_subscription(
            Funnel,
            'funnel_action',
            self.msg_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.last_msg = np.zeros(3)
        self.do_action = False

    def msg_callback(self, msg):
        for i in range(3):
            print("msg part", i, msg.action[i])
            if self.last_msg[i] != msg.action[i]:
                self.last_msg[i] = msg.action[i]
                self.do_action = True
        print(self.last_msg)
        if self.do_action == True:
            self.funnel.all_actions(self.last_msg, msg.action[3])
            self.do_action = False
        
    def destroy_node(self) -> bool:
        self.funnel.__del__
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    funnel = FunnelSubscriber()

    rclpy.spin(funnel)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    funnel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()