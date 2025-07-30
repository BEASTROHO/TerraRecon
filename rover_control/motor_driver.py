import RPi.GPIO as GPIO
import time

class MotorDriver:
    def __init__(self, left_pins, right_pins):
        self.left_pins = left_pins
        self.right_pins = right_pins
        GPIO.setmode(GPIO.BCM)
        for pin in left_pins + right_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

    def move_forward(self):
        GPIO.output(self.left_pins[0], GPIO.HIGH)
        GPIO.output(self.left_pins[1], GPIO.LOW)
        GPIO.output(self.right_pins[0], GPIO.HIGH)
        GPIO.output(self.right_pins[1], GPIO.LOW)

    def stop(self):
        for pin in self.left_pins + self.right_pins:
            GPIO.output(pin, GPIO.LOW)

    def cleanup(self):
        GPIO.cleanup()
