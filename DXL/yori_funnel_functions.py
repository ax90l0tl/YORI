import numpy as np
import DXLManager2 as DM

class yori_funnel(object):
    def __init__(self, port='/dev/ttyUSB0'):
        print("initializing ...")
        self.all = [1, 2, 3, 4, 5]
        self.up = [1, 3] 
        self.turn = [2]
        self.tilt = [4, 5]
        self.yori_funnel = DM.Dynamixel(devicename=port)
        self.yori_funnel.set_joint_torque_enable()
        self.yori_funnel.extended_position_enable()
        self.yori_funnel.initialize(100)
        self.dt = DM.dt

    # def too_fast(self, motors, target_theta, num_vp):
    #     target_theta = np.array([target_theta])
    #     current = self.yori_funnel.read_motors(motors) * (1/DM.position_constant)
    #     print("current", current)
    #     delta = (np.abs(target_theta - np.mean(current)))/360
    #     speed = delta num_vp
    #     print("delta", speed)
    #     if speed >= DM.max_rps:
    #         print( num_vp too low, will proceed at maximum speed allowed")
    #      num_vp = 1.02*delta/DM.max_rps
    #         if num_vp > 200:
    #          num_vp = 5
    #         print num_vp)
    #         return num_vp)
    #     else:
    #         return num_vp)

    def funnel_fold_up(self, num_vp=100):
        print("folding up funnel ...")
        # num_vp = self.too_fast(self.tilt, 0, num_vp)
        self.yori_funnel.operate_motors_relative(self.tilt, np.array([-90, -90]), num_vp)

    def funnel_fold_down(self, num_vp=100):
        print("folding down funnel ...")
        # num_vp = self.too_fast(self.tilt, 90, num_vp)
        self.yori_funnel.operate_motors_relative(self.tilt, np.array([90, 90]), num_vp)

    def funnel_forward(self, num_vp=100):
        print("bringing funnel forward ...")
        # gear ratio of motor:gear is 19:100
        target = -(100/19)*180
        # num_vp = self.too_fast(self.turn, target, num_vp)
        self.yori_funnel.operate_motors_relative(self.turn, np.array([target]), num_vp)

    def funnel_backward(self, num_vp=100):
        print("bringing funnel backwards ...")
        # gear ratio of motor:gear is 19:100
        target = (100/19)*180
        # num_vp = self.too_fast(self.turn, target, num_vp)
        self.yori_funnel.operate_motors_relative(self.turn, np.array([target]), num_vp)

    # distance in mm
    def funnel_up_down(self, distance, num_vp=50):
        print("going up ...")
        target = -(distance/32)*360
        # num_vp = self.too_fast(self.up, target, num_vp)
        self.yori_funnel.operate_motors_relative(self.up, np.array([target]), num_vp)

    def __del__(self):
        print("shutting down ...")
        # self.yori_funnel.initialize(100)
        self.yori_funnel.set_joint_torque_disable()