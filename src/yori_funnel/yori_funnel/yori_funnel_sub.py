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
        self.last_msg = np.zeros(4)  # variable to store message
        self.do_action = False

    # first number is the z height
    # second number is the rotation
    # third number is the tilt

    # fourth number is a specific set of actions for each type of plate
    # Code:
    # 1. Tray position
    # 2. Fry basket position
    # 3. Before putting in pot
    # 4. After putting in pot
    # 5. Plating

    def msg_callback(self, msg):
        for i in range(4):
            if self.last_msg[i] != msg.action[i]:
                self.last_msg[i] = msg.action[i]
                self.do_action = True
        if self.do_action == True:
            if msg.action[3] != 0:
                print("Going to preset position: ", msg.action[3])
                self.funnel.preset_pos(msg.action[3])
            else:
                print("Doing", self.last_msg, " ...")
                self.funnel.all_actions(self.last_msg)
            self.do_action = False

    def destroy_node(self) -> bool:
        self.funnel.__del__
        return super().destroy_node()


def main(args=None):
    try:
        rclpy.init(args=args)
        funnel = FunnelSubscriber()
        rclpy.spin(funnel)
    except KeyboardInterrupt:
        funnel.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
