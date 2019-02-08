# tr_hat_bridge
This package is responsible for communication with Turtle Hat (RPi shield)

## Nodes

### tr_hat_bridge
Exposes ROS API for communication with Turtle Hat

#### Subscribed Topics

* **`~motors`** ([tr_hat_msgs/MotorPower])

	Sets motor power levels described by float values between -1.0 and 1.0

#### Services

* **`~get_battery`** ([tr_hat_msgs/GetBattery])

	Returns battery voltage

* **`~get_firmware_ver`** ([tr_hat_msgs/GetFirmwareVer])

	Returns version of installed firmware on Turtle Hat

#### Parameters

* **`~device`** (`string`, default: `"/dev/ttyAMA0"`)

	Serial device connected to Turtle Hat

[tr_hat_msgs/MotorPower]: https://github.com/TurtleRover/tr_ros/blob/master/tr_hat_msgs/msg/MotorPower.msg
[tr_hat_msgs/GetBattery]: https://github.com/TurtleRover/tr_ros/blob/master/tr_hat_msgs/srv/GetBattery.srv
[tr_hat_msgs/GetFirmwareVer]: https://github.com/TurtleRover/tr_ros/blob/master/tr_hat_msgs/srv/GetFirmwareVer.srv