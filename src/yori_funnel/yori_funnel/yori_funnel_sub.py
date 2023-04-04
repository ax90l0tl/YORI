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
        self.last_msg = np.zeros(3) #variable to store message
        self.do_action = False

    def msg_callback(self, msg):
        #first number is the z height
        #second number is the rotation
        #third number is the tilt
        #fourth number is the number of via points (less means faster)
        for i in range(3):
            if self.last_msg[i] != msg.action[i]:
                self.last_msg[i] = msg.action[i]
                self.do_action = True
        if self.do_action == True:
            print("Doing", self.last_msg, " ...")
            # fold up funnel if we need to rotate the funnel
            if self.last_msg[1] != msg.action[i]:
                self.funnel.funnel_fold_up()
            self.funnel.funnel_up_down(self.last_msg[0])
            self.funnel.funnel_spin(self.last_msg[1])
            self.funnel.funnel_tilt(self.last_msg[2])
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
