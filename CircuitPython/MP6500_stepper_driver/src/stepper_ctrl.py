import board
import digitalio
import pwmio

class stepper:
    def __init__(self, in1, in2, step, dir):
        self.in1 = pwmio.PWMOut(in1, duty_cycle=0, frequency=500)
        self.in2 = digitalio.DigitalInOut(in2)
        self.step = pwmio.PWMOut(step, duty_cycle=0, frequency=500)
        self.dir = digitalio.DigitalInOut(dir)
        self.in2.direction = digitalio.Direction.OUTPUT
        self.dir.direction = digitalio.Direction.OUTPUT
        self.in2.drive_mode = digitalio.DriveMode.OPEN_DRAIN
        self.dir.drive_mode = digitalio.DriveMode.PUSH_PULL
        
    def __init__(self, in1, in2, step, dir, freq):
        stepper(self, in1, in2, step, dir)
        self.in1.frequency = freq
        self.step.frequency = freq

    def run(self, speed, dir):
        if dir >= 0:
            self.dir.pull = digitalio.Pull.UP
        else:
            self.dir.pull = digitalio.Pull.DOWN
        self.step.duty_cycle = int(speed * 65535) 

