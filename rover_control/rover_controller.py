from motor_driver import MotorDriver
import json

# Load motor pin config
with open('rover_control/config.json', 'r') as f:
    config = json.load(f)

left_pins = config['left_motor_pins']
right_pins = config['right_motor_pins']

# Initialize motor driver
driver = MotorDriver(left_motor_pins=left_pins, right_motor_pins=right_pins)

# Movement sequence
driver.move_forward(2)
driver.turn_left(1)
driver.move_forward(2)
driver.stop()
