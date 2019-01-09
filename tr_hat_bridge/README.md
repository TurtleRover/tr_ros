# tr_hat_bridge
This package is responsible for communication with Turtle Hat (RPi shield)

## Nodes

### tr_hat_bridge
Exposes ROS API for communication with Turtle Hat

#### Subscribed Topics

* **`~motors`** ([tr_msgs/MotorPayload])

	Sends the motor payload values described by 8-bit integers. Values between 0-127 (0x00-0x7F) move the motors forward and values 128-255 (0x80-0xFF) move them backward

#### Services

* **`~get_battery`** ([tr_msgs/GetBattery])

	Returns battery ADC reading

[tr_msgs/MotorPayload]: https://github.com/TurtleRover/tr_ros/blob/master/tr_msgs/msg/MotorPayload.msg
[tr_msgs/GetBattery]: https://github.com/TurtleRover/tr_ros/blob/master/tr_msgs/srv/GetBattery.srv