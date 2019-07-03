import rospy

import struct
import RPi.GPIO as GPIO
from time import sleep


def clamp(value, mn, mx):
    return max(min(value, mx), mn)


def power_to_motor_payload(power):
    p = clamp(power, -1.0, 1.0)
    value = int(round(p * 0x7F))
    if value < 0:
        value = -value + 0x7F
    return value


def servo_angle_to_duty(angle):
    value = int((angle / 180.0) * 3450 + 1300)
    duty = struct.pack(">H", value)
    return duty


def reset_STM():
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT)
        GPIO.output(18, GPIO.LOW)
        sleep(0.5)
        GPIO.cleanup()
    except RuntimeError:
        rospy.logwarn("Could not reset STM on Turtle Hat. No access to GPIO pins. "
                      "Try running as root!")