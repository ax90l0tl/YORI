import os, sys
import time
import numpy as np

# sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))))
from DynamixelSDK.python.src import dynamixel_sdk as DXLSDK
# import src.yori_control.yori_control.library.dynamixel_sdk as DXLSDK
from Chrono import *
#from src.yori_utility.yori_utility.Chrono import *
# import python.src.dynamixel_sdk as DXLSDK

XM = {}

xm430_position_offset_180 = 2048
position_constant = (xm430_position_offset_180/180)
xm430_position_offset_90 = xm430_position_offset_180/2

# Dynamixel will rotate between this value
DXL_MIN_POSITION_VALUE = [0]

# and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MAX_POSITION_VALUE = [2097151]

# Velocity limit (0~1023)
DXL_MAX_VELOCITY_VALUE = [1000]
DXL_MAX_VELOCITY_PROFILE = [1000]

# limit
DXL_MAX_VELOCITY_VALUE = DXL_MAX_VELOCITY_VALUE

# profile  limit (less than velocity & acceleration limit) # DXL_PROFILE_VELOCITY = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,]
DXL_PROFILE_VELOCITY = DXL_MAX_VELOCITY_PROFILE

# DXL_ID = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
DXL_ID = [1, 2, 3, 4, 5]

#  Dynamixel XM430 moving status threshold
DXL_MOVING_STATUS_THRESHOLD = [10]

# PID gain (0 ~ 16,383)
P_gain = [100]
I_gain = [10]
D_gain = [0]

ESC_ASCII_VALUE = 0x1b  # for ESC key to escape out from the operation

COMM_SUCCESS = 0  # Communication Success result value
COMM_TX_FAIL = -1001  # Communication Tx Failed

class Dynamixel(object):
    def __init__(self, baudrate=115200, devicename='/dev/ttyUSB0', control_table=XM, protocol_version=2.0): #type (Dynamixel, dict) -> None
        self.ctable = control_table

        self.DXL_LOWORD = DXLSDK.DXL_LOWORD
        self.DXL_HIWORD = DXLSDK.DXL_HIWORD
        self.DXL_LOBYTE = DXLSDK.DXL_LOBYTE
        self.DXL_HIBYTE = DXLSDK.DXL_HIBYTE

        self.baudrate = baudrate
        self.devicename = devicename

        self.packet_handler = DXLSDK.PacketHandler(protocol_version)
        self.port_handler = DXLSDK.PortHandler(devicename)

        self.packet_writeTxRx   = {1:self.packet_handler.write1ByteTxRx,   2:self.packet_handler.write2ByteTxRx,   4:self.packet_handler.write4ByteTxRx}
        self.packet_readTxRx    = {1:self.packet_handler.read1ByteTxRx ,   2:self.packet_handler.read2ByteTxRx ,   4:self.packet_handler.read4ByteTxRx }
        # Not use until needed (maybe when speed up)
        # self.packet_writeTx     = {1:self.packet_handler.write1ByteTx  ,   2:self.packet_handler.write2ByteTx  ,   4:self.packet_handler.write4ByteTx  }
        # self.packet_readRx      = {1:self.packet_handler.read1ByteRx   ,   2:self.packet_handler.read2ByteRx   ,   4:self.packet_handler.read4ByteRx   }
        # self.packet_readTx      = {1:self.packet_handler.read1ByteTx   ,   2:self.packet_handler.read2ByteTx   ,   4:self.packet_handler.read4ByteTx   }

# Do we need this code? ---------------------------------------------

        if os.name == 'nt':
            import msvcrt
            def getch():
                return msvcrt.getch().decode()
        else:
            import sys, tty, termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            def getch():
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch

# -------------------------------------------------------------------
        # Open port
        if self.port_handler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()
        
        # if not self.port_handler.openPort():
        #     print("Failed to open the port")
        #     return

        # Set port baudrate
        if self.port_handler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate through port")
        else:
            print("Failed to change the baudrate through port")
            print("Press any key to terminate...")
            getch()
            quit()

        # if not self.port_handler.setBaudRate(self.baudrate):
        #     print("Failed to change the baudrate through port")
        #     return

        # Set Status return level
        self.write('STATUS_RETURN_LEVEL', zip(DXL_ID, (2, )*len(DXL_ID)) )
        
        # Operating mode
        # self.write('OPERATING_MODE', zip(DXL_ID, (3, )*len(DXL_ID)) )
        self.write('OPERATING_MODE', zip(DXL_ID, (4, 3)) ) # 3 - Position Control Mode, 4 - Extended Position Control Mode
        
        # Change drive mode (CCW -> CW)
        # self.write('DRIVE_MODE', zip(DXL_ID, [1]) )

        # Set position limit
        self.write('MAX_POSITION_LIMIT'    , zip(DXL_ID, DXL_MAX_POSITION_VALUE))
        self.write('MIN_POSITION_LIMIT'    , zip(DXL_ID, DXL_MIN_POSITION_VALUE))

        # Set velocity limit
        self.write('VELOCITY_LIMIT'        , zip(DXL_ID, DXL_MAX_VELOCITY_VALUE))

        # Set profile velocity
        self.write('PROFILE_VELOCITY'      , zip(DXL_ID, DXL_PROFILE_VELOCITY))

        # Set position P gain, I gain, D gain - all 2 byte (0~16383)- P only 254 for unit communication
        self.write('POSITION_P_GAIN'       , zip(DXL_ID, P_gain))
        self.write('POSITION_I_GAIN'       , zip(DXL_ID, I_gain))
        self.write('POSITION_D_GAIN'       , zip(DXL_ID, D_gain))

        # Set control mode
        self.control_mode = 'position'
    
    def initialize(self, num_vp):
        init_theta_0 = np.array([0, 0, 0, 0, 0]) # in degrees
        #init_theta_1 = np.array([206.2]) # in degrees
        self.operate_motors(DXL_ID, init_theta_0, num_vp)
        #self.operate_motor_single(DXL_ID[1],init_theta_1, num_vp)

    def vps_gen_js(self, start_pos, end_pos, num_vp):
        """
        Generate via points in joint space from present joint angle to desired joint angle

        param start_pos: initial joint angles
        type start_pos: 1xi list

        param end_pos: desired joint angles
        type end_pos: 1xi list

        param num_vp: the number of via points including end_point
        type num_vp: int (num_vp >= 1)

        return: vps_js in joint space
        rtype: nxi numpy array

        """
        vps_js = np.linspace(start_pos, end_pos, num_vp, axis=-1)
        # print(vps_js)

        return vps_js

    # def vps_gen_js_single(self, start_pos, end_pos, num_vp):
        """
        Generate via points in joint space from present joint angle to desired joint angle

        param start_pos: initial joint angles
        type start_pos: 1xi list

        param end_pos: desired joint angles
        type end_pos: 1xi list

        param num_vp: the number of via points including end_point
        type num_vp: int (num_vp >= 1)

        return: vps_js in joint space
        rtype: nxi numpy array

        """
        vps_js = np.linspace(start_pos, end_pos, num_vp)
        return vps_js

    def set_joint_torque_enable(self): 
        self.write('TORQUE_ENABLE', zip(DXL_ID, (1, )*len(DXL_ID)))

    def set_joint_torque_disable(self):
        self.write('TORQUE_ENABLE', zip(DXL_ID, (0, )*len(DXL_ID)))    

    def extended_position_control_enable(self):
        self.write('OPERATING_MODE', zip(DXL_ID, (4, )*len(DXL_ID)))

    def extended_position_control_disable(self):
        self.write('OPERATING_MODE', zip(DXL_ID, (3, )*len(DXL_ID)))

    # def operate_motor_single(self, id, goal_theta, num_vp):
        # present_theta = self.read_motors(id)
        # # print(present_theta)
        # # Transfer target theta
        # new_goal_theta = goal_theta * position_constant
        # present_theta = present_theta * position_constant
        # # print(present_theta)
        # # print(new_goal_theta)

        # vps = self.vps_gen_js(present_theta, new_goal_theta, num_vp)
        # # print(vps)

        # dt = 1.e-3
        # t0 = time.time()
        # index_step = 0

        # while index_step < num_vp:
            
        #     # print(vps[index_step])
        #     self.write('GOAL_POSITION', zip([id], np.rint(vps[0]).astype(int)))

        #     # print(index_step)
        #     # print(vps[index_step])

        #     index_step += 1
        #     # busy loop to sync
        #     wait(dt, t0)        # wait for dt from t0 until t0+dt
        #     t1 = time.time()
        #     # print("Time = {}".format(t1-t0))
        #     t0 = time.time()    # update t0, the new t0 ~= the last t0 + dt

    def operate_motors(self, ids, target_theta, num_vp):
        present_theta = self.read_motors(ids)

        # Transfer target theta
        new_target_theta = target_theta * position_constant
        present_theta = present_theta * position_constant

        print("current:", present_theta)
        print("target:", new_target_theta)

        vps = self.vps_gen_js(present_theta, new_target_theta, num_vp)
        print(vps)
        dt = 1.e-3
        t0 = time.time()
        index = 0

        # print(len(vps))

        while index < num_vp:
            # print(index)
            print(list(zip(ids, np.rint(vps[:, index]).astype(int))))
            self.write('GOAL_POSITION', zip(ids, np.floor(vps[:, index]).astype(int)))

            # update index
            index += 1

            # busy loop to sync
            wait(dt, t0)        # wait for dt from t0 until t0+dt
            t1 = time.time()
            # print("Time = {}".format(t1-t0))
            t0 = time.time()    # update t0, the new t0 ~= the last t0 + dt

    def read_motors(self, ids):
        read_th = self.read('PRESENT_POSITION', ids)
        print(read_th)
        current_pos = np.zeros(len(ids))
        for i in range(len(ids)):
            current_pos[i] = read_th[i][0]
        print(current_pos)
        #current_pos_0 = read_th[0][0]
        # current_pos_1 = read_th[1][0]
        # position_constant = 180.0/xm430_position_offset_180
        # if current_pos_1 > 0x7fffffff:
        #     current_pos_1 = current_pos_1 - 2088959
        # current_pos = current_pos * position_constant
        # print(current_pos * int(position_constant))
        # current_pos_1 = current_pos_1 * position_constant
        # current_pos = np.array([current_pos])

        return current_pos#, current_pos_1])
    
    def add_parameter_storage(self, ids):
        [self.position_reader.addParam(id) for id in ids]

    def allocate_goal_position(self, command, *args):
        command = list(map(int, command))
        self.cmd_position = []
        for i in command:
            self.cmd_position.append([self.DXL_LOBYTE(self.DXL_LOWORD(i)), self.DXL_HIBYTE(self.DXL_LOWORD(i)), self.DXL_LOBYTE(self.DXL_HIWORD(i)), self.DXL_HIBYTE(self.DXL_HIWORD(i))])
        return self.cmd_position
        
    def set_command_position(self, id_command, *args):
        [self.position_writer.addParam(id, cmd) for id, cmd in id_command]
        comm_res = self.position_writer.txPacket()
        if comm_res != COMM_SUCCESS:
            print("{}".format(self.packet_handler.getTxRxResult(comm_res)))
        self.position_writer.clearParam()

    def write(self, addr_key, id_command, *args):
        # command = [(id1, cmd1), (id, cmd), (id, cmd), ...]
        return [self.packet_writeTxRx[self.ctable[addr_key]['SIZE']](self.port_handler, id, self.ctable[addr_key]['ADDR'], cmd) for (id, cmd) in id_command]

    def write_debug(self, addr_key, id_command, *args):
        # command = [(id1, cmd1), (id, cmd), (id, cmd), ...]
        return [self._check_error(self.packet_writeTxRx[self.ctable[addr_key]['SIZE']](self.port_handler, id, self.ctable[addr_key]['ADDR'], cmd), id, addr_key) for (id, cmd) in id_command]

    def read(self, addr_key, ids, *args):
        # command = [id, id, id, ...]
        return [self.packet_readTxRx[self.ctable[addr_key]['SIZE']](self.port_handler, id, self.ctable[addr_key]['ADDR']) for id in ids]

    def read_debug(self, addr_key, ids, *args):
        # command = [id, id, id, ...]
        return [self._check_error(self.packet_readTxRx[self.ctable[addr_key]['SIZE']](self.port_handler, id, self.ctable[addr_key]['ADDR']), id, addr_key) for id in ids]

    def _check_error(self, comm_res, id, addr_key):
        res, err = comm_res
        if res != COMM_SUCCESS:
            print("({},{}): {}}".format(id, addr_key, self.packet_handler.getTxRxResult(res)) )
        elif err != 0:
            print("({},{}): {}}".format(id, addr_key, self.packet_handler.getRxPacketError(err)) )
        return comm_res

    def _check_error_sync(self, result, id):
        if result != True:
            print('[ID{:03d}] groupSync failed'.format(id))
        return result


### CONTROL TABLE DEFINITION
# reference: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
XM['MODEL_NUMBER']            = {'KEY':'MODEL_NUMBER'          , 'ADDR':  0, 'SIZE':2, 'ACCESS':'R' }
XM['MODEL_INFORMATION']       = {'KEY':'MODEL_INFORMATION'     , 'ADDR':  2, 'SIZE':4, 'ACCESS':'R' }
XM['FIRMWARE_VERSION']        = {'KEY':'FIRMWARE_VERSION'      , 'ADDR':  6, 'SIZE':1, 'ACCESS':'R' }
XM['ID']                      = {'KEY':'ID'                    , 'ADDR':  7, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          0,        252)}
XM['BAUD_RATE']               = {'KEY':'BAUD_RATE'             , 'ADDR':  8, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          0,          7)}
XM['RETURN_DELAY_TIME']       = {'KEY':'RETURN_DELAY_TIME'     , 'ADDR':  9, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          0,        254)}
XM['DRIVE_MODE']              = {'KEY':'DRIVE_MODE'            , 'ADDR': 10, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          0,          5)}
XM['OPERATING_MODE']          = {'KEY':'OPERATING_MODE'        , 'ADDR': 11, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          0,         16)}
XM['PROTOCOL_TYPE']           = {'KEY':'PROTOCOL_TYPE'         , 'ADDR': 13, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          1,          2)}
XM['HOMING_OFFSET']           = {'KEY':'HOMING_OFFSET'         , 'ADDR': 20, 'SIZE':4, 'ACCESS':'RW', 'RANGE':(   -1044479,    1044479)}
XM['MOVING_THRESHOLD']        = {'KEY':'MOVING_THRESHOLD'      , 'ADDR': 24, 'SIZE':4, 'ACCESS':'RW', 'RANGE':(          0,       1023)}
XM['TEMPERATURE_LIMIT']       = {'KEY':'TEMPERATURE_LIMIT'     , 'ADDR': 31, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          0,        100)}
XM['MAX_VOLTAGE_LIMIT']       = {'KEY':'MAX_VOLTAGE_LIMIT'     , 'ADDR': 32, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(         95,        160)}
XM['MIN_VOLTAGE_LIMIT']       = {'KEY':'MIN_VOLTAGE_LIMIT'     , 'ADDR': 34, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(         95,        160)}
XM['PWM_LIMIT']               = {'KEY':'PWM_LIMIT'             , 'ADDR': 36, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(          0,        885)}
XM['CURRENT_LIMIT']           = {'KEY':'CURRENT_LIMIT'         , 'ADDR': 38, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(          0,       1193)}
XM['VELOCITY_LIMIT']          = {'KEY':'VELOCITY_LIMIT'        , 'ADDR': 44, 'SIZE':4, 'ACCESS':'RW', 'RANGE':(          0,       1023)}
XM['MAX_POSITION_LIMIT']      = {'KEY':'MAX_POSITION_LIMIT'    , 'ADDR': 48, 'SIZE':4, 'ACCESS':'RW', 'RANGE':(          0,       4095)}
XM['MIN_POSITION_LIMIT']      = {'KEY':'MIN_POSITION_LIMIT'    , 'ADDR': 52, 'SIZE':4, 'ACCESS':'RW', 'RANGE':(          0,       4095)}
XM['SHUTDOWN']                = {'KEY':'SHUTDOWN'              , 'ADDR': 63, 'SIZE':1, 'ACCESS':'RW'}
XM['TORQUE_ENABLE']           = {'KEY':'TORQUE_ENABLE'         , 'ADDR': 64, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          0,          1)}
XM['LED_RED']                 = {'KEY':'LED'                   , 'ADDR': 65, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          0,          1)}
XM['STATUS_RETURN_LEVEL']     = {'KEY':'STATUS_RETURN_LEVEL'   , 'ADDR': 68, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          0,          2)}
XM['REGISTERED_INSTRUCTION']  = {'KEY':'REGISTERED_INSTRUCTION', 'ADDR': 69, 'SIZE':1, 'ACCESS':'R' }
XM['HARDWARE_ERROR_STATUS']   = {'KEY':'HARDWARE_ERROR_STATUS' , 'ADDR': 70, 'SIZE':1, 'ACCESS':'R' }
XM['VELOCITY_I_GAIN']         = {'KEY':'VELOCITY_I_GAIN'       , 'ADDR': 76, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(          0,      16383)}
XM['VELOCITY_P_GAIN']         = {'KEY':'VELOCITY_P_GAIN'       , 'ADDR': 78, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(          0,      16383)}
XM['POSITION_D_GAIN']         = {'KEY':'POSITION_D_GAIN'       , 'ADDR': 80, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(          0,      16383)}
XM['POSITION_I_GAIN']         = {'KEY':'POSITION_I_GAIN'       , 'ADDR': 82, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(          0,      16383)}
XM['POSITION_P_GAIN']         = {'KEY':'POSITION_P_GAIN'       , 'ADDR': 84, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(          0,      16383)}
XM['FEEDFORWARD_2ND_GAIN']    = {'KEY':'FEEDFORWARD_2ND_GAIN'  , 'ADDR': 88, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(          0,      16383)}
XM['FEEDFORWARD_1ST_GAIN']    = {'KEY':'FEEDFORWARD_1ST_GAIN'  , 'ADDR': 90, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(          0,      16383)}
XM['BUS_WATCHDOG']            = {'KEY':'BUS_WATCHDOG'          , 'ADDR': 98, 'SIZE':1, 'ACCESS':'RW', 'RANGE':(          0,        127)}
XM['GOAL_PWM']                = {'KEY':'GOAL_PWM'              , 'ADDR':100, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(       -885,        885)}    #           -PWM Limit(36) ~ PWM Limit(36)
XM['GOAL_CURRENT']            = {'KEY':'GOAL_CURRENT'          , 'ADDR':102, 'SIZE':2, 'ACCESS':'RW', 'RANGE':(      -1193,       1193)}    # *     -Current Limit(38) ~ Current Limit(38)
XM['GOAL_VELOCITY']           = {'KEY':'GOAL_VELOCITY'         , 'ADDR':104, 'SIZE':4, 'ACCESS':'RW', 'RANGE':(      -1023,       1023)}    # *    -Velocity Limit(44) ~ Velocity Limit(44)
XM['PROFILE_ACCELERATION']    = {'KEY':'PROFILE_ACCELERATION'  , 'ADDR':108, 'SIZE':4, 'ACCESS':'RW', 'RANGE':(          0,      32767)}
XM['PROFILE_VELOCITY']        = {'KEY':'PROFILE_VELOCITY'      , 'ADDR':112, 'SIZE':4, 'ACCESS':'RW', 'RANGE':(          0,      32767)}
XM['GOAL_POSITION']           = {'KEY':'GOAL_POSITION'         , 'ADDR':116, 'SIZE':4, 'ACCESS':'RW', 'RANGE':(      -4095,       4095)}    # * Min Position Limit(52) ~ Max Position Limit(48)
XM['REALTIME_TICK']           = {'KEY':'REALTIME_TICK'         , 'ADDR':120, 'SIZE':2, 'ACCESS':'R' , 'RANGE':(          0,      32767)}
XM['MOVING']                  = {'KEY':'MOVING'                , 'ADDR':122, 'SIZE':1, 'ACCESS':'R' }
XM['MOVING_STATUS']           = {'KEY':'MOVING_STATUS'         , 'ADDR':123, 'SIZE':1, 'ACCESS':'R' }
XM['PRESENT_PWM']             = {'KEY':'PRESENT_PWM'           , 'ADDR':124, 'SIZE':2, 'ACCESS':'R' }
XM['PRESENT_CURRENT']         = {'KEY':'PRESENT_CURRENT'       , 'ADDR':126, 'SIZE':2, 'ACCESS':'R' }
XM['PRESENT_VELOCITY']        = {'KEY':'PRESENT_VELOCITY'      , 'ADDR':128, 'SIZE':4, 'ACCESS':'R' }
XM['PRESENT_POSITION']        = {'KEY':'PRESENT_POSITION'      , 'ADDR':132, 'SIZE':4, 'ACCESS':'R' }
XM['VELOCITY_TRAJECTORY']     = {'KEY':'VELOCITY_TRAJECTORY'   , 'ADDR':136, 'SIZE':4, 'ACCESS':'R' }
XM['POSITION_TRAJECTORY']     = {'KEY':'POSITION_TRAJECTORY'   , 'ADDR':140, 'SIZE':4, 'ACCESS':'R' }
XM['PRESENT_INPUT_VOLTAGE']   = {'KEY':'PRESENT_INPUT_VOLTAGE' , 'ADDR':144, 'SIZE':2, 'ACCESS':'R' }
XM['PRESENT_TEMPERATURE']     = {'KEY':'PRESENT_TEMPERATURE'   , 'ADDR':146, 'SIZE':1, 'ACCESS':'R' }