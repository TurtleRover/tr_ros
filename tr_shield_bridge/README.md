# tr_shield_bridge
This package provides a bridge of communication between ROS and Turtle Hut (RPi shield)

## Nodes

### tr_shield_bridge

#### Subscribed Topics

* **`tr_shield_bridge/motors`** ([tr_msgs/MotorPayload])

	Sends the motor payload values described by 8-bit integers. Values between 0-127 (0x00-0x7F) move the motors forward and values 128-255 (0x80-0xFF) move them backward

#### Services

* **`tr_shield_bridge/get_battery`** ([tr_msgs/GetBattery])

	Returns battery ADC reading from Turtle Hat

[tr_msgs/MotorPayload]: https://github.com/TurtleRover/tr_ros/blob/master/tr_msgs/msg/MotorPayload.msg
[tr_msgs/GetBattery]: https://github.com/TurtleRover/tr_ros/blob/master/tr_msgs/srv/GetBattery.srv