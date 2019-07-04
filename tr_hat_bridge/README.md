# tr_hat_bridge
This package is responsible for communication with Turtle Hat (RPi shield)

## Nodes

### tr_hat_bridge
Exposes ROS API for communication with Turtle Hat

#### Subscribed Topics

* **`~motors`** ([std_msgs/Float32MultiArray])

	Sets motor power levels described by float values between -1.0 and 1.0

* **`~servoX/angle`** ([std_msgs/Int16])

	Sets servo angle on channel X (1,2 or 3)

#### Published Topics

* **`battery`** ([std_msgs/Float32])

	Current battery voltage

* **`firmware_version`** ([std_msgs/String])

	Current version of the firmware on Turtle Hat

#### Parameters

* **`~device`** (`string`, default: `"/dev/ttyAMA0"`)

	Serial device connected to Turtle Hat

* **`~battery_pub_rate`** (`float`, default: `1.0`)

	The frequency in Hz at which battery voltage is published 

* **`~servoX_min_angle`** (`int`, default: `-90`)

	Minimal angle on servo channel X

* **`~servoX_max_angle`** (`int`, default: `90`)

	Maximal angle on servo channel X

* **`~servoX_min_duty`** (`int`, default: `2400`)

	Servo duty on channel X corresponding to minimal angle (0 => 0% PWM duty, 48000 => 100% PWM duty)

* **`~servoX_max_duty`** (`int`, default: `4800`)

	Servo duty on channel X corresponding to maximal angle

[std_msgs/String]: http://docs.ros.org/api/std_msgs/html/msg/String.html
[std_msgs/Int16]: http://docs.ros.org/api/std_msgs/html/msg/Int16.html
[std_msgs/Float32]: http://docs.ros.org/api/std_msgs/html/msg/Float32.html
[std_msgs/Float32MultiArray]: http://docs.ros.org/api/std_msgs/html/msg/Float32MultiArray.html