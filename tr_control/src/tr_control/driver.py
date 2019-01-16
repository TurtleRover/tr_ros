

class DriverBase(object):
    def __init__(self, wheel_radius=0.06, wheel_track=0.33):
        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.wheel_radius = wheel_radius
        self.wheel_track = wheel_track

    def set_motors(self, linear, angular):
        raise NotImplementedError()


class DriverStraight(DriverBase):

    def set_motors(self, linear, angular):

        if abs(linear) >= abs(angular):

            wheel_speed = linear / self.wheel_radius

            self.wheel_speeds = [wheel_speed] * 4

        else:

            wheel_left_speed = (-angular * (self.wheel_track / 2)) / self.wheel_radius
            wheel_right_speed = (angular * (self.wheel_track / 2)) / self.wheel_radius

            self.wheel_speeds = [wheel_left_speed, wheel_right_speed] * 2


class DriverDifferential(DriverBase):

    def set_motors(self, linear, angular):

        wheel_left_speed = (linear - angular * (self.wheel_track / 2)) / self.wheel_radius
        wheel_right_speed = (linear + angular * (self.wheel_track / 2)) / self.wheel_radius

        self.wheel_speeds = [wheel_left_speed, wheel_right_speed] * 2
