import time
import board
import DRV8874_motor_driver

motor = DRV8874_motor_driver.motor_pwm(board.D5, board.D6)

def message_received(msg):
    print("Received message:", msg.data)    

rclpy.init(name="teensy_node")
subscriber = rclpy.create_subscription(String, "Motor_ctrl", message_received)
rclpy.spinOnce()
time.sleep(1)

