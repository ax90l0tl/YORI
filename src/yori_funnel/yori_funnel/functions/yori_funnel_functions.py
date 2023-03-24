import numpy as np
from yori_funnel.DXL import DXLManager2 as DM

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
        self.yori_funnel.initialize()

    def funnel_fold_up(self, num_vp=50):
        print("folding up funnel ...")
        self.yori_funnel.operate_motors_relative(self.tilt, np.array([-90, -90]), num_vp)

    def funnel_fold_down(self, num_vp=50):
        print("folding down funnel ...")
        self.yori_funnel.operate_motors_relative(self.tilt, np.array([90, 90]), num_vp)

    def funnel_tilt(self, target, num_vp=50):
        new_target = np.ones(2) * target
        self.yori_funnel.operate_motors_relative(self.tilt, new_target, num_vp)

    def funnel_forward(self, num_vp=100):
        print("bringing funnel forward ...")
        # gear ratio of motor:gear is 19:100
        target = -(100/19)*180
        self.yori_funnel.operate_motors_relative(self.turn, np.array([target]), num_vp)

    def funnel_backward(self, num_vp=100):
        print("bringing funnel backwards ...")
        # gear ratio of motor:gear is 19:100
        target = (100/19)*180
        self.yori_funnel.operate_motors_relative(self.turn, np.array([target]), num_vp)

    # distance in mm
    def funnel_up_down(self, distance, num_vp=50):
        print("going up ...")
        target = -(distance/32)*360
        self.yori_funnel.operate_motors_rel(self.up, np.array([target]), num_vp)

    def all_actions(self, target, num_vp=100):
        # target is an array of three numbers
        # target[0] is the vertical position of the funnel
        # target[1] is the rotation of the funnel in degrees
        # target[2] is the tilt of the funnel
        target[0] = -(target[0]/32)*360
        target[1] = target[1] * (100/19)
        if target[2] > 90:
            target[2] = 90
        elif target[2] < -90:
            target[2] = -90
        self.yori_funnel.operate_motors_abs_rel(self.all, [target[0], target[1], target[0], target[2], target[2]], num_vp)

    def __del__(self):
        print("shutting down ...")
        # self.yori_funnel.initialize(100)
        self.yori_funnel.set_joint_torque_disable()