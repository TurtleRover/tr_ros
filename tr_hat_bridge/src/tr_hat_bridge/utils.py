import struct


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
