import board
import digitalio
import pwmio
import math
import time

class stepper:
    def __init__(self, MS1, MS2, step, dir):


        self.MS1 = digitalio.DigitalInOut(MS1)
        self.MS1.direction = digitalio.Direction.OUTPUT
        # self.MS1.drive_mode = digitalio.DriveMode.PUSH_PULL

        self.MS2 = digitalio.DigitalInOut(MS2)
        self.MS2.direction = digitalio.Direction.OUTPUT
        # self.MS2.drive_mode = digitalio.DriveMode.PUSH_PULL

        # self.in1 = pwmio.PWMOut(step, duty_cycle=0, frequency=1000)

        # self.in1 = digitalio.DigitalInOut(in1)
        # self.in1.direction = digitalio.Direction.OUTPUT
        # self.in1.drive_mode = digitalio.DriveMode.OPEN_DRAIN
        # self.in1.value = True

        # self.in2 = digitalio.DigitalInOut(in2)
        # self.in2.direction = digitalio.Direction.OUTPUT
        # self.in2.drive_mode = digitalio.DriveMode.OPEN_DRAIN
        # self.in2.value = False

        self.step = digitalio.DigitalInOut(step)
        self.step.direction = digitalio.Direction.OUTPUT
        self.step.drive_mode = digitalio.DriveMode.PUSH_PULL
        self.step.value = False
        # self.step = pwmio.PWMOut(step, duty_cycle=0, variable_frequency=True)

        self.dir = digitalio.DigitalInOut(dir)
        self.dir.direction = digitalio.Direction.OUTPUT
        self.dir.drive_mode = digitalio.DriveMode.PUSH_PULL

    def step_size(self, step):
        if step == 1:
            self.MS1.value = False
            self.MS2.value = False
        elif step == 2:
            self.MS1.value = True
            self.MS2.value = False
        elif step == 4:
            self.MS1.value = False
            self.MS2.value = True
        else:
            self.MS1.value = True
            self.MS2.value = True

    def run(self, speed, step_amount):
        if speed != 0:
            if speed > 0:
                self.dir.value = True
            else:
                self.dir.value = False
                
            self.step_size(step_amount)
            pulse_freq = abs(speed)*499*step_amount
            #1999
            #1999
            #1999

            #print(pulse_freq)
            self.step.value = True
            time.sleep(1/pulse_freq)
            self.step.value = False
        else:
            self.step.value = False


    def stop(self):
        self.run(0, 1)