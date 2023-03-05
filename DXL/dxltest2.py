import os
import time
import numpy as np
import DXLManager2 as DM

# import yori_funnel.DXLManager2 as DM
# import Utility.DXLManager2 as DM

if __name__ == "__main__":
    XMDXL = DM.Dynamixel()
    # print(YORIDXL.read_motors())
    XMDXL.extended_position_enable()
    XMDXL.set_joint_torque_enable()
    
    all = [1, 2, 3, 4, 5]
    up = [1, 3] 
    turn = [2]
    tilt = [4, 5]
    target = np.array([0, 0, 0, 0, 0])
    # position_constant = [(501923/180)]
    # print( target * position_constant)
    # a = zip(id, target)
    # print(a)
    # XMDXL.extended_position_control_enable()
    XMDXL.initialize(50)
    print(XMDXL.read_motors(all))
    XMDXL.check_step_size(all, [90, 90, 90, 90, 90], 10)
    XMDXL.operate_motors(tilt, np.array([180, 180]), 130)
    # XMDXL.operate_motors(up, np.array([-90]), 50)
    # time.sleep(10)
    # XMDXL.operate_motors([1], np.array([0]), 100)
    # time.sleep(2)
    # XMDXL.operate_motors([1], np.array([3*360]), 100)
    # time.sleep(2)
    # XMDXL.operate_motors(tilt, np.array([0]), 100)
    # time.sleep(2)
    # XMDXL.operate_motors(tilt, np.array([5*360]), 5*100)
    # time.sleep(2)
    #XMDXL.operate_motors([1], np.array([0]), 100)
    #time.sleep(2)
    # XMDXL.operate_motors(tilt, target[3], 100)
    # XMDXL.operate_motors(up, np.array([720, 720]), 100)
    # XMDXL.operate_motors([5], target[1], 3)
    # XMDXL.operate_motor_single(4, target[1], 3)
    # XMDXL.operate_motor_single(2, target[1], 3)
    #time.sleep(1)
    print(XMDXL.read_motors(all))

    #XMDXL.operate_motors(target,100)
    # XMDXL.operate_motors(target,100)
    #time.sleep(1)
    #XMDXL.operate_motor_single(4, target[0], 100)
    #print(XMDXL.read_motors())

    #XMDXL.initialize(100)
    #time.sleep(2)
    # print(XMDXL.read_motors())
    
    #XMDXL.operate_motors(target,100)

    XMDXL.set_joint_torque_disable()