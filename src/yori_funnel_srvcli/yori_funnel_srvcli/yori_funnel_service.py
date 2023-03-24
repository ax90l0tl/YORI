import rclpy
from rclpy.node import Node
from yori_funnel_srvcli.src import yori_funnel_functions
from custom_msg_srv.srv import Funnel

class FunnelService(Node):
    def __init__(self):
        super().__init__('funnel_service')
        self.srv = self.create_service(Funnel, 'funnel', self.funnel_callback)
        self.funnel = yori_funnel_functions.yori_funnel()

    def funnel_callback(self, request, response):
        response.time_taken = self.action(request)
        return response

    def action(self, request):
        func = {
            "FOLD_UP" : self.funnel.funnel_up_down,
            "FOLD_DOWN" : self.funnel.funnel_up_down,
            "TURN_CW" : self.funnel.funnel_up_down,
            "TURN_CCW" : self.funnel.funnel_up_down,
            "ORIENTATION" : self.funnel.funnel_up_down,
            "UP" : self.funnel.funnel_up_down,
            "DOWN" : self.funnel.funnel_up_down,
            "HEIGHT": self.funnel.funnel_up_down
        }
        return func.get(request.action)

    def main(args=None):
        rclpy.iniy(args=args)
        funnel_service = FunnelService()
        rclpy.spin(funnel_service)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()