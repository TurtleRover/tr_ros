# tr_teleop
This package contains nodes that provide Rover's teleoperation via joystick or keyboard

## Nodes

### tr_teleop_key
This node allows teleoperation using keyboard input

#### Published Topics

* **`cmd_vel_stamped`** ([geometry_msgs/TwistStamped])

    velocity command

#### Parameters

* **`~speed_linear`** (`float`, default: `0.4`)

    maximum linear speed in m/s.

* **`~accel_linear`** (`float`, default: `0.8`)

    linear acceleration and decelaration in m/s^2

* **`~speed_angular`** (`float`, default: `3.0`)

    maximum angular speed in r/s.

* **`~accel_angular`** (`float`, default: `6.0`)

    angular acceleration and decelaration in r/s^2

* **`~key_timeout`** (`float`, default: `0.3`)

    if no key is received for a specified time, sets target speed to 0. If set to `0.0`, timeout checking will be disabled

### tr_teleop_joy
This node allows teleoperation using joystick input. It receives joystick input messages (e.g. from [joy_node]) and translates each message to velocity command

#### Subscribed Topics

* **`joy`** ([sensor_msgs/Joy])

    input from joystick

#### Published Topics

* **`cmd_vel_stamped`** ([geometry_msgs/TwistStamped])

    velocity command

#### Parameters

* **`axis_linear`** (`int`, default: `1`)

    joystick axis for controlling linear velocity

* **`axis_angular`** (`int`, default: `3`)

    joystick axis for controlling angular velocity

* **`scale_linear`** (`float`, default: `0.4`)

    amount to scale the joystick input for the linear velocity output

* **`scale_angular`** (`float`, default: `3.0`)

    amount to scale the joystick input for the angular velocity output

## Launch files

### joy.launch

Launches [joy_node] with autorepeat parameter and `tr_teleop_joy` node with scale parameters and joy mapping from config file `config/joy_mapping.yaml`


[geometry_msgs/TwistStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
[joy_node]: http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick
[sensor_msgs/Joy]: http://docs.ros.org/jade/api/sensor_msgs/html/msg/Joy.html