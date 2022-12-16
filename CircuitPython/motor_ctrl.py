import time
import board
import pwmio

# frequency = 500 is default
motor1 = pwmio.PWMOut(board.D5, duty_cycle=0, frequency=500)
motor2 = pwmio.PWMOut(board.D6, duty_cycle=0, frequency=500)


def motor_run(speed, mode):
    # Speed is from -1 to 1
    # positive is CW
    # negative is CCW
    on = 65535
    off = 0
    if mode == "coast":
        if speed >= 0:
            motor1.duty_cycle = int(speed * 65535)
            motor2.duty_cycle = off
        else:
            motor1.duty_cycle = off
            motor2.duty_cycle = int(-speed * 65535)
    elif mode == "brake":
        if speed >= 0:
            motor2.duty_cycle = int(65535 - speed * 65535)
            motor1.duty_cycle = on
        elif speed < 0:
            motor2.duty_cycle = on
            motor1.duty_cycle = int(65535 + speed * 65535)


while True:
    # PWM range is from 0 - 65535
    motor_run(0, "coast")
    time.sleep(0.01)
