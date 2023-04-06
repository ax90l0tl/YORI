import numpy as np
import time
from FUNNEL_SYSTEM import *
from yori_funnel.DXL import DXLManager2 as DM

# -13000 is the stir-fry pot before putting it in
# -5000 is the stir fry pot after putting it in
# -3000 is the tray
# -13500 is the table
# -8000 is fry basket

heights = np.array([-3000, -8000, -5000, -13500, -13500]) * \
    (1/DM.position_constant)
print(heights)


class yori_funnel(object):
    def __init__(self, port='/dev/ttyUSB0'):
        print("initializing ...")
        # Different Motor groupings for commands
        self.all = [1, 2, 3, 4, 5]
        self.up = [1, 3]
        self.turn = [2]
        self.tilt = [4, 5]
        self.no_tilt = [1, 2, 3]

        self.tilt_lim_down = -10
        self.tilt_lim_up = -90

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
            self.all_actions(np.array([105, 0, 10]))
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
        # ratio of vertical movemoent is 32mm for every rotation
        target = -(distance/32)*360
        # height limits
        if target[0] > 0:
            target = 0
        elif target[0] < -105:
            target[0] = 105
        return target
    # spin conversions and limits

    def convert_spin(self, spin):
        # ratio of funnel to motor is 100:19
        target = (100/19)*spin
        # rotaion limits
        if target[1] < 0:
            target = 0
        elif target[1] > 180:
            target = 180
        return target
    # tilt limits

    def convert_tilt(self, tilt):
        # -90 is down
        if tilt < 90:
            tilt = 90
        elif self.get_height() >= 50:
            if tilt > 10:
                tilt = 10
                self.tilt_lim_up = 10
                print("Funnel cannot exceed 10 degrees at this height")
            # 0 is up
        elif tilt > 0:
            self.tilt_lim_up = 0
            tilt = 0
        return (tilt)

    def funnel_fold_up(self):
        # print("folding up funnel ...")
        if self.abs == False:
            self.yori_funnel.operate_motors_abs_rel(
                self.tilt, np.array([-self.tilt_lim_up, -self.tilt_lim_up]))
        else:
            self.yori_funnel.operate_motors_abs(
                self.tilt, np.array([-self.tilt_lim_up, -self.tilt_lim_up]))

    def funnel_fold_down(self):
        # print("folding down funnel ...")
        if self.abs == False:
            self.yori_funnel.operate_motors_abs_rel(
                self.tilt, np.array([-self.tilt_lim_down, -self.tilt_lim_down]))
        else:
            self.yori_funnel.operate_motors_abs(self.tilt, np.array(
                [-self.tilt_lim_down, -self.tilt_lim_down]))

    def funnel_tilt(self, target):
        #  negative so that zero is folded up and 90 is folded down on the ROS message side
        target = self.convert_tilt(target)
        new_target = -np.ones(2) * target
        if self.abs == False:
            self.yori_funnel.operate_motors_abs_rel(self.tilt, -new_target)
        else:
            self.yori_funnel.operate_motors_abs(self.tilt, -new_target)

    def funnel_spin(self, spin):
        # print("bringing funnel backwards ...")
        # gear ratio of motor:gear is 19:100
        target = self.convert_spin(spin)
        if self.abs == False:
            self.yori_funnel.operate_motors_abs_rel(
                self.turn, np.array([target]))
        else:
            self.yori_funnel.operate_motors_abs(self.turn, np.array([target]))

    # distance in mm
    def funnel_height(self, distance):
        # print("going up ...")
        target = self.convert_height(distance)
        new_target = -np.ones(2) * target
        if self.abs == False:
            self.yori_funnel.operate_motors_abs_rel(
                self.up, new_target)
        else:
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
        if spin != 0:
            self.funnel_fold_up()
            self.yori_funnel.operate_motors_abs_rel(
                self.no_tilt, [height, spin, height])
            self.tilt(self.tilt, [-tilt, -tilt])
        else:
            self.yori_funnel.operate_motors_abs_rel(
                self.all, [height, spin, height, -tilt, -tilt])

    def preset_pos(self, id):
        var = actions['PRESET_HEIGHTS'][id]
        target = var.get('POSITION')
        print('Going to ', var.get('NAME'), 'at position', target)
        self.all_actions(target)


def __del__(self):
    print("shutting down ...")
    #  go to default position
    self.all_actions([105, 0, 10])
    #  sleep time to make sure default position is reached
    time.sleep(5)
    self.yori_funnel.set_joint_torque_disable()
