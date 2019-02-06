# tr_hat_bridge
This package is responsible for communication with Turtle Hat (RPi shield)

## Nodes

### tr_hat_bridge
Exposes ROS API for communication with Turtle Hat

#### Subscribed Topics

* **`~motors`** ([tr_hat_msgs/MotorPayload])

	Sends the motor payload values described by 8-bit integers.  
	Values between 0-127 (0x00-0x7F) move the motors forward and values 128-255 (0x80-0xFF) move them backward

#### Services

* **`~get_battery`** ([tr_hat_msgs/GetBattery])

	Returns battery voltage

* **`~get_firmware_ver`** ([tr_hat_msgs/GetFirmwareVer])

	Returns version of installed firmware on Turtle Hat

#### Parameters

* **`~device`** (`string`, default: `"/dev/ttyAMA0"`)

	Serial device connected to Turtle Hat

[tr_hat_msgs/MotorPayload]: https://github.com/TurtleRover/tr_ros/blob/master/tr_hat_msgs/msg/MotorPayload.msg
[tr_hat_msgs/GetBattery]: https://github.com/TurtleRover/tr_ros/blob/master/tr_hat_msgs/srv/GetBattery.srv
[tr_hat_msgs/GetFirmwareVer]: https://github.com/TurtleRover/tr_ros/blob/master/tr_hat_msgs/srv/GetFirmwareVer.srv