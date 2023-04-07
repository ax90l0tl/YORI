import numpy as np
import time
from yori_funnel.dictionary.FUNNEL_SYSTEM import *
from yori_funnel.DXL import DXLManager2 as DM


class yori_funnel(object):
    def __init__(self, port='/dev/ttyUSB0'):
        print("initializing ...")
        # Different Motor groupings for commands
        self.all = [1, 2, 3, 4, 5]
        self.up = [1, 3]
        self.turn = [2]
        self.tilt = [4, 5]
        self.no_tilt = [1, 2, 3]

        self.tilt_lim_down = 10
        self.tilt_lim_up = 90

        self.yori_funnel = DM.Dynamixel(devicename=port)
        self.yori_funnel.extended_position_enable()
        self.yori_funnel.set_joint_torque_enable()
        self.abs = False
        current = self.yori_funnel.read_motors(self.all)
        if np.abs(current[0] - current[2]) > 250 or np.abs(current[3] - current[4]) > 100:
            # if the up/down or the tilt motors have too big of an offset from each other then dont proceed with the absolute method
            # you will break the system!
            print("Funnel not zeroed")
            self.__del__()
        else:
            # make 0 the current position
            # use absolute positioning
            self.yori_funnel.set_zero([0, 0, 0, 0, 0])
            self.preset_pos(7)
            self.abs = True

    def get_height(self):
        current = self.yori_funnel.read_motors(self.up)
        current = current * 1/DM.position_constant
        current = -current * 32/360
        return np.mean(current)

    def get_rotation(self):
        current = self.yori_funnel.read_motors(self.turn)
        current = current * 1/DM.position_constant
        current = current * 19/100
        return current

    def get_tilt(self):
        current = self.yori_funnel.read_motors(self.tilt)
        current = current * 1/DM.position_constant
        return np.mean(current)

    # height conversion and limits
    def convert_height(self, distance):
        # height limits
        if distance < 0:
            distance = 0
        elif distance > 105:
            distance = 105
        # ratio of vertical movemoent is 32mm for every rotation
        target = -(distance/32)*360
        return target
    # spin conversions and limits

    def convert_spin(self, spin):
        # rotaion limits
        if spin < 0:
            spin = 0
        elif spin > 180:
            spin = 180
        # ratio of funnel to motor is 100:19
        target = (100/19)*spin
        return target
    # tilt limits

    def convert_tilt(self, tilt):
        # 90 is down
        if tilt > 90:
            tilt = 90
        elif self.get_height() >= 50:
            if tilt < 10:
                tilt = 10
                self.tilt_lim_up = 10
                print("Funnel cannot exceed 10 degrees at this height")
            # 0 is up
        elif tilt < 0:
            self.tilt_lim_up = 0
            tilt = 0
        return (tilt)

    def will_collide(self, height):
        if height >= 50 or self.get_height() >= 50:
            self.tilt_lim_up = 10
            print("Funnel cannot exceed 10 degrees at this height")
        else:
            self.tilt_lim_up = 0
        print(self.tilt_lim_up)

    def funnel_fold_up(self):
        # print("folding up funnel ...")
        self.convert_tilt(self.tilt_lim_up)
        if self.abs == True:
            self.yori_funnel.operate_motors_abs(
                self.tilt, np.array([-self.tilt_lim_up, -self.tilt_lim_up]))

    def funnel_fold_down(self):
        # print("folding down funnel ...")
        if self.abs == True:
            self.yori_funnel.operate_motors_abs(self.tilt, np.array(
                [-self.tilt_lim_down, -self.tilt_lim_down]))

    def funnel_tilt(self, target):
        #  negative so that zero is folded up and 90 is folded down on the ROS message side
        target = self.convert_tilt(target)
        new_target = -np.ones(2) * target
        if self.abs == True:
            self.yori_funnel.operate_motors_abs(self.tilt, new_target)

    def funnel_spin(self, spin):
        # print("bringing funnel backwards ...")
        # gear ratio of motor:gear is 19:100
        target = self.convert_spin(spin)
        if self.abs == True:
            self.yori_funnel.operate_motors_abs(self.turn, np.array([target]))

    # distance in mm
    def funnel_height(self, distance):
        # print("going up ...")
        target = self.convert_height(distance)
        new_target = -np.ones(2) * target
        if self.abs == True:
            self.yori_funnel.operate_motors_abs(
                self.up, new_target)

    def all_actions(self, target):
        # target is an array of three numbers
        # target[0] is the vertical position of the funnel
        # target[1] is the rotation of the funnel in degrees
        # target[2] is the tilt of the funnel

        height = self.convert_height(target[0])
        spin = self.convert_spin(target[1])
        tilt = self.convert_tilt(target[2])
        self.will_collide(target[0])
        if self.abs == True:
            self.funnel_fold_up()
            self.yori_funnel.operate_motors_abs(self.no_tilt, [height, spin, height])
            self.funnel_tilt(target[2])
        self.yori_funnel.read_motors(self.all)

    def preset_pos(self, id):
        var = actions['PRESET_HEIGHTS'][id]
        target = var.get('POSITION')
        print('Going to ', var.get('NAME'), 'at position', target)
        self.all_actions(target)


    def __del__(self):
        print("shutting down ...")
        #  go to default position
        if self.abs == True:
            self.preset_pos(7)
            #  sleep time to make sure default position is reached
            time.sleep(1)
        # self.yori_funnel.set_joint_torque_disable()
