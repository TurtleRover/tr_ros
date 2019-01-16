

class Controller(object):
    def __init__(self, max_wheel_speed=6.0):
        self.payload = [0x00, 0x00, 0x00, 0x00]  # RR, RL, FR, FL
        self.max_wheel_speed = max_wheel_speed

    @staticmethod
    def convert_motor_power_to_payload(power):
        value = int(round(power * 0x7F))
        if value < 0:
            value = -value + 0x7F
        return value

    def set_wheels(self, wheel_speeds):
        for i, wheel_speed in enumerate(wheel_speeds):
            wheel_speed = min(self.max_wheel_speed, max(-self.max_wheel_speed, wheel_speed))
            motor_power = wheel_speed / self.max_wheel_speed
            self.payload[i] = Controller.convert_motor_power_to_payload(motor_power)
