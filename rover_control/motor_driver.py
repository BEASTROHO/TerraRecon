# rover_control/motor_driver.py

import RPi.GPIO as GPIO
import time

class MotorDriver:
    """
    Controls the left and right motors of the TerraRecon rover.
    Uses H-bridge motor driver connected to Raspberry Pi GPIO pins.
    """

    def __init__(self, left_motor_pins, right_motor_pins, pwm_freq=100):
        self.left_motor_pins = left_motor_pins  # [IN1, IN2, EN]
        self.right_motor_pins = right_motor_pins  # [IN3, IN4, EN]
        GPIO.setmode(GPIO.BCM)

        # Setup motor pins
        for pin in left_motor_pins[:2] + right_motor_pins[:2]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        GPIO.setup(left_motor_pins[2], GPIO.OUT)
        GPIO.setup(right_motor_pins[2], GPIO.OUT)

        self.left_pwm = GPIO.PWM(left_motor_pins[2], pwm_freq)
        self.right_pwm = GPIO.PWM(right_motor_pins[2], pwm_freq)

        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def move_forward(self, speed=50):
        GPIO.output(self.left_motor_pins[0], GPIO.HIGH)
        GPIO.output(self.left_motor_pins[1], GPIO.LOW)
        GPIO.output(self.right_motor_pins[0], GPIO.HIGH)
        GPIO.output(self.right_motor_pins[1], GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(speed)
        self.right_pwm.ChangeDutyCycle(speed)

    def move_backward(self, speed=50):
        GPIO.output(self.left_motor_pins[0], GPIO.LOW)
        GPIO.output(self.left_motor_pins[1], GPIO.HIGH)
        GPIO.output(self.right_motor_pins[0], GPIO.LOW)
        GPIO.output(self.right_motor_pins[1], GPIO.HIGH)
        self.left_pwm.ChangeDutyCycle(speed)
        self.right_pwm.ChangeDutyCycle(speed)

    def turn_left(self, speed=50):
        GPIO.output(self.left_motor_pins[0], GPIO.LOW)
        GPIO.output(self.left_motor_pins[1], GPIO.HIGH)
        GPIO.output(self.right_motor_pins[0], GPIO.HIGH)
        GPIO.output(self.right_motor_pins[1], GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(speed)
        self.right_pwm.ChangeDutyCycle(speed)

    def turn_right(self, speed=50):
        GPIO.output(self.left_motor_pins[0], GPIO.HIGH)
        GPIO.output(self.left_motor_pins[1], GPIO.LOW)
        GPIO.output(self.right_motor_pins[0], GPIO.LOW)
        GPIO.output(self.right_motor_pins[1], GPIO.HIGH)
        self.left_pwm.ChangeDutyCycle(speed)
        self.right_pwm.ChangeDutyCycle(speed)

    def stop(self):
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)
        for pin in self.left_motor_pins[:2] + self.right_motor_pins[:2]:
            GPIO.output(pin, GPIO.LOW)

    def cleanup(self):
        self.stop()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()
