import board
import time
import DRV8874_motor_driver

motor = DRV8874_motor_driver.motor_pwm(board.D5, board.D6, 500)

while True:
    motor.run(1, "coast")
    time.sleep(0.01)