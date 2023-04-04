import numpy as np
import time
from yori_funnel.DXL import DXLManager2 as DM

class yori_funnel(object):
    def __init__(self, port='/dev/ttyUSB0'):
        print("initializing ...")
        self.all = [1, 2, 3, 4, 5]
        self.up = [1, 3] 
        self.turn = [2]
        self.tilt = [4, 5]
        self.yori_funnel = DM.Dynamixel(devicename=port)
        self.yori_funnel.extended_position_enable()
        self.yori_funnel.set_joint_torque_enable()
        self.abs = False
        current = self.yori_funnel.read_motors(self.all)
        if np.abs(current[0] - current[2]) > 250 or np.abs(current[3] - current[4]) > 100:
            # if the up/down or the tilt motors have too big of an offset from each other then dont proceed with the absolute method
            # you will break the system!
            print("Funnel not zeroed, setting current position to zero")
            # use absolute relative positioning
            self.yori_funnel.initialize()
            # fold funnel up because startup position will have the funnel down
            self.funnel_fold_up()
            # get initial posiiton after funnel is up
            self.yori_funnel.initialize()
        else:
            # make 0 the current position
            # use absolute positioning
            self.yori_funnel.set_zero([0, 0, 0, 0, 0])
            self.funnel_fold_up()
            self.all_actions(np.array([0, 0, 0]))
            self.abs = True

    def funnel_fold_up(self):
        # print("folding up funnel ...")
        if self.abs == False:
            self.yori_funnel.operate_motors_abs_rel(self.tilt, np.array([0, 0]))
        else:
            self.yori_funnel.operate_motors_abs(self.tilt, np.array([0, 0]))

    def funnel_fold_down(self):
        # print("folding down funnel ...")
        if self.abs == False:
            self.yori_funnel.operate_motors_abs_rel(self.tilt, np.array([-90, -90]))
        else:
            self.yori_funnel.operate_motors_abs(self.tilt, np.array([-90, -90]))

    # generalized tilt function
    def funnel_tilt(self, target):
        #  negative so that zero is folded up and 90 is folded down on the ROS message side
        new_target = -np.ones(2) * target
        if self.abs == False:
            self.yori_funnel.operate_motors_abs_rel(self.tilt, new_target)
        else:
            self.yori_funnel.operate_motors_abs(self.tilt, new_target)

    def funnel_spin(self, spin):
        # print("bringing funnel backwards ...")
        # gear ratio of motor:gear is 19:100
        target = (100/19)*spin
        if self.abs == False:
            self.yori_funnel.operate_motors_abs_rel(self.turn, np.array([target]))
        else:
            self.yori_funnel.operate_motors_abs(self.turn, np.array([target]))

    # distance in mm
    def funnel_up_down(self, distance):
        # print("going up ...")
        target = -(distance/32)*360
        if self.abs == False:
            self.yori_funnel.operate_motors_abs_rel(self.up, np.array([target, target]))
        else:
            self.yori_funnel.operate_motors_abs(self.up, np.array([target, target]))

    def all_actions(self, target):
        # target is an array of three numbers
        # target[0] is the vertical position of the funnel
        # target[1] is the rotation of the funnel in degrees
        # target[2] is the tilt of the funnel
        #ratio of vertical movemoent is 32mm for every rotation
        target[0] = -(target[0]/32)*360
        #height limits
        # if target[0] < 0:
        #     target = 0
        # elif target[0] > 100:
        #     target[0] = 100
        #rotaion limits
        if target[1] < 0:
            target = 0
        elif target[1] > 180:
            target = 180
        #gear ratio of funnel to motor is 100:19
        target[1] = target[1] * (100/19)
        #tilt limits
        if target[2] > 0:
            target[2] = 0
        elif target[2] < -90:
            target[2] = -90
        self.yori_funnel.operate_motors_abs_rel(self.all, [target[0], target[1], target[0], target[2], target[2]])

    def __del__(self):
        print("shutting down ...")
        # fold funnel up to avoid collision with the arms
        self.funnel_fold_up()
        #  go to default position
        self.all_actions([0, 0, 0])
        #  sleep time to make sure default position is reached
        time.sleep(5)
        self.yori_funnel.set_joint_torque_disable()