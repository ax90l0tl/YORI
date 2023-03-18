import os
import time
import numpy as np
import DXLManager as DM
# import yori_funnel.DXLManager2 as DM
# import Utility.DXLManager2 as DM

if __name__ == "__main__":
    XMDXL = DM.Dynamixel()
    # print(YORIDXL.read_motors())
    XMDXL.set_joint_torque_enable()
    
    id = [2, 4]
    target = np.array([0, 0])
    # position_constant = [(501923/180)]
    # print( target * position_constant)
    # a = zip(id, target)
    # print(a)

    XMDXL.initialize(2)
    time.sleep(2)
    print(XMDXL.read_motors())

    XMDXL.operate_motor_single(id[0],target[0],100)
    # XMDXL.operate_motors(target,100)
    time.sleep(2)
    print(XMDXL.read_motors())

    XMDXL.initialize(2)
    time.sleep(2)
    # print(XMDXL.read_motors())
    
    XMDXL.operate_motors(target,100)

    XMDXL.set_joint_torque_disable()