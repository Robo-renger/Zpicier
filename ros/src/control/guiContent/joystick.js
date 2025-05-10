// ROS Connection



const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'  // Change this to your ROSBridge WebSocket URL
});

ros.on('connection', function () {
    console.log('Connected to ROS');
});

ros.on('error', function (error) {
    console.log('Error connecting to ROS:', error);
});

ros.on('close', function () {
    console.log('Connection to ROS closed');
});

// Define the topic to publish joystick data
const joystickTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/joystick',
    messageType: 'msgs/Joystick'
});

function sendJoystickData() {
    const gamepad = scanGamepads();
    if (gamepad) {
        const axes = gamepad.axes.map(value => parseFloat(value.toFixed(2)));
        const buttons = gamepad.buttons.map(btn => btn.pressed ? true : false);

        const buttonNames = [
            "button_x", "button_o", "button_rect", "button_tri",
            "button_l1", "button_r1", "button_l2", "button_r2",
            "button_l3", "button_r3", "button_top", "button_bot",
            "button_left", "button_right", "button_options", "button_share"
        ];

        const buttonStates = [
            buttons[0], buttons[1], buttons[2], buttons[3],
            buttons[4], buttons[5], buttons[6], buttons[7],
            buttons[10], buttons[11], buttons[12], buttons[13],
            buttons[14], buttons[15], buttons[9], buttons[8]
        ];

        const msg = new ROSLIB.Message({
            left_x_axis: axes[0],
            left_y_axis: axes[1],
            right_x_axis: axes[2],
            right_y_axis: axes[3],
            button_names: buttonNames,
            button_states: buttonStates
        });

        console.log("Sending joystick data (gamepad):", msg);
        joystickTopic.publish(msg);
    } else {
        // Fallback: get axes and buttons from manual input
        const rawAxes = document.getElementById("axes").value;   // e.g. "0.1,-0.2,0.0,0.5"
        const rawButtons = document.getElementById("buttons").value; // e.g. "button_l1:true,button_x:false"

        if (!rawAxes || !rawButtons) return;

        const axesValues = rawAxes.split(",").map(s => parseFloat(s.trim())); // [lx, ly, rx, ry]
        const mappedNames = [];
        const boolValues = [];

        rawButtons.split(",").forEach(pair => {
            const [name, val] = pair.split(":").map(s => s.trim());
            mappedNames.push(name);
            boolValues.push(val.toLowerCase() === "true");
        });

        const msg = new ROSLIB.Message({
            left_x_axis: axesValues[0] || 0.0,
            left_y_axis: axesValues[1] || 0.0,
            right_x_axis: axesValues[2] || 0.0,
            right_y_axis: axesValues[3] || 0.0,
            button_names: mappedNames,
            button_states: boolValues
        });

        console.log("Sending joystick data (manual):", msg);
        joystickTopic.publish(msg);
    }
}



// Function to detect gamepads
function scanGamepads() {
    const gamepads = navigator.getGamepads();
    console.log(gamepads)
    return gamepads[0]; // Use the first connected gamepad
}


setInterval(sendJoystickData, 100);

var pidTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/constants',
    messageType: 'std_msgs/Float32MultiArray'
});

// Function to send PID constants via ROS topic
function sendPIDConstants() {
    var kp1 = parseFloat(document.getElementById("kp1").value) || 0;
    var ki1 = parseFloat(document.getElementById("ki1").value) || 0;
    var kd1 = parseFloat(document.getElementById("kd1").value) || 0;
    var kp2 = parseFloat(document.getElementById("kp2").value) || 0;
    var ki2 = parseFloat(document.getElementById("ki2").value) || 0;
    var kd2 = parseFloat(document.getElementById("kd2").value) || 0;
    var kp3 = parseFloat(document.getElementById("kp3").value) || 0;
    var ki3 = parseFloat(document.getElementById("ki3").value) || 0;
    var kd3 = parseFloat(document.getElementById("kd3").value) || 0;

    var pidMessage = new ROSLIB.Message({
        data: [kp1, ki1, kd1, kp2, ki2, kd2, kp3, ki3, kd3]
    });

    pidTopic.publish(pidMessage);
    console.log("Sent PID constants:", pidMessage.data);
}