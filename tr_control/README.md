# tr_control
Drive system for Turtle Rover

## Nodes

### tr_control
This node takes velocity commands as Twist messages and communicates with [tr_hat_bridge] to drive the Rover

#### Subscribed Topics

* **`cmd_vel`** ([geometry_msgs/Twist])

    Target velocity of the Rover. Only linear.x (m/s) and angular.z (r/s) are used.

* **`cmd_vel_stamped`** ([geometry_msgs/TwistStamped])

    Stamped velocity commands. Should not be published with `cmd_vel` at the same time


#### Published Topics

* **`tr_hat_bridge/motors`** ([tr_hat_msgs/MotorPower])

    Motor power values sent to Turtle Hat

#### Parameters

* **`~wheel_radius`** (`float`, default: `0.06`)

    Radius of Rover's wheel in meters

* **`~wheel_track`** (`float`, default: `0.33`)

    Length of the wheel track, i.e., distance in meters between left and right wheels

* **`~max_wheel_speed`** (`float`, default: `6.0`)

    Maximum wheel rotation speed in radians per second. It is currently used by the controller to evaluate motor power value based on wheel speed.

* **`~differential_drive`** (`bool`, default: `true`)

    If set to true the driver will support differential steering, i.e., driving along a curve. Otherwise the driver will only support driving straight or turning in place

* **`~input_timeout`** (`float`, default: `0.5`)

    Input timeout in seconds. The controller will stop the motors if it doesn't receive a command for a specified time. If set to 0, the controller won't check for a timeout.

* **`~timestamp_check`** (`float`, default: `0.0`)

    Velocity commands with timestamps that are older than this value will be ignored. Only affects `cmd_vel_stamped` messages. Requires synchronized time between Rover and host machine.  
    If set to 0, timestamp checking will be disabled.


[tr_hat_bridge]: https://github.com/TurtleRover/tr_ros/tree/master/tr_hat_bridge
[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[geometry_msgs/TwistStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
[tr_hat_msgs/MotorPower]: https://github.com/TurtleRover/tr_ros/blob/master/tr_hat_msgs/msg/MotorPower.msg