import time
import board
import supervisor
from DRV8874_motor_driver.src import DRV8874_motor_driver

motor = DRV8874_motor_driver.motor_pwm(board.D5, board.D6, 500)

# topic = ''
# if supervisor.runtime.serial_connected:
#     while topic == '':
#         print(f"motor_ctrl\n\r")
#         topic = str(input().strip())
#         if topic != '':
#             print(f"{topic}\n\r")
#             break

while supervisor.runtime.serial_connected:
    if supervisor.runtime.serial_bytes_available:
         sub = str(input().strip())
         split = sub.split(",")
         motor.run(float(split[0]), split[1])
         print(f"Received: {sub}\n\r")

motor.run(0, "coast")
         

