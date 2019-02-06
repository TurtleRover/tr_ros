var twist;
var speedLinear;
var speedAngular;
var cmdVelPub;
var manager;
var ros;

function initVelocityPublisher() {
    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    // Init topic object
    cmdVelPub = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    // Register publisher within ROS system
    cmdVelPub.advertise();
}

function initTeleopKeyboard() {
    var body = document.getElementsByTagName('body')[0];
    body.addEventListener('keydown', function(e) {
        switch(e.keyCode) {
            case 65: //left
                twist.angular.z = speedAngular;
                break;
            case 68: //right
                twist.angular.z = -speedAngular;
                break;
            case 87: ///up
                twist.linear.x = speedLinear;
                break;
            case 83: //down
                twist.linear.x = -speedLinear;
        }
    });
    body.addEventListener('keyup', function(e) {
        switch(e.keyCode) {
            case 65: //left
            case 68: //right
                twist.angular.z = 0;
                break;
            case 87: ///up
            case 83: //down
                twist.linear.x = 0;
        }
    });
}

function createJoystick() {

    if (manager == null) {
        joystickContainer = document.getElementById('joystick');

        manager = nipplejs.create({
            zone: joystickContainer,
            position: { left: 50 + '%', top: 105 + 'px' },
            mode: 'static',
            size: 200,
            color: '#0066ff',
            restJoystick: true
        });

        manager.on('move', function (evt, nipple) {

            var lin = Math.sin(nipple.angle.radian) * nipple.distance * 0.01;
            var ang = -Math.cos(nipple.angle.radian) * nipple.distance * 0.01;

            twist.linear.x = lin * speedLinear;
            twist.angular.z = ang * speedAngular;
        });

        manager.on('end', function () {
            twist.linear.x = 0
            twist.angular.z = 0
        });
    }
}

function initSliders() {
    $('#lin-slider').slider({
        tooltip: 'show',
        min: 0,
        max: 0.4,
        step: 0.01,
        value: 0.2
    });
    $('#lin-slider').on("slide", function(slideEvt) {
        speedLinear = slideEvt.value;
    });
    speedLinear = 0.2

    $('#ang-slider').slider({
        tooltip: 'show',
        min: 0,
        max: 6.0,
        step: 0.1,
        value: 3.0
    });
    $('#ang-slider').on("slide", function(slideEvt) {
        speedAngular = slideEvt.value;
    });
    speedAngular = 3.0
}


function publishTwist() {
    //console.log("twist " + twist.linear.x + " " + twist.angular.z)
    cmdVelPub.publish(twist)
}

window.onload = function () {
    var robot_hostname = window.location.hostname;

    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_hostname + ":9090"
    });

    initSliders();
    initVelocityPublisher();
    initTeleopKeyboard();
    createJoystick();

    setInterval(() => publishTwist(), 50);

}