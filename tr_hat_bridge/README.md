# tr_hat_bridge
This package is responsible for communication with Turtle Hat (RPi shield)

## Nodes

### tr_hat_bridge
Exposes ROS API for communication with Turtle Hat

#### Subscribed Topics

* **`~motors`** ([std_msgs/Float32MultiArray])

	Sets motor power levels described by float values between -1.0 and 1.0

* **`~servoX/angle`** ([std_msgs/Float32])

	Sets servo angle on channel X (1,2 or 3)

#### Services

* **`~get_battery`** ([tr_hat_msgs/GetBattery])

	Returns battery voltage

* **`~get_firmware_ver`** ([tr_hat_msgs/GetFirmwareVer])

	Returns version of installed firmware on Turtle Hat

#### Parameters

* **`~device`** (`string`, default: `"/dev/ttyAMA0"`)

	Serial device connected to Turtle Hat

[std_msgs/Float32]: http://docs.ros.org/api/std_msgs/html/msg/Float32.html
[std_msgs/Float32MultiArray]: http://docs.ros.org/api/std_msgs/html/msg/Float32MultiArray.html
[tr_hat_msgs/GetBattery]: https://github.com/TurtleRover/tr_ros/blob/master/tr_hat_msgs/srv/GetBattery.srv
[tr_hat_msgs/GetFirmwareVer]: https://github.com/TurtleRover/tr_ros/blob/master/tr_hat_msgs/srv/GetFirmwareVer.srv