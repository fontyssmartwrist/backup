// Create a ROSTOPIC with the name '/web_orientation_goal' and message type 
// Quaternion from geometry_msgs
var sendQuatTopic = new ROSLIB.Topic({
ros : ros,
name : '/web_orientation_goal',
messageType : 'geometry_msgs/Quaternion'
});

// Make a object that holds the quaternion message
var quaternionMessage = new ROSLIB.Message({
x: 0.0,
y: 0.0,
z: 0.0,
w: 0.0
});

// Setup the object that has the name and message type 
var imu2Sensor = new ROSLIB.Topic({
ros : ros,
name : '/tfsensors/imu1',
messageType : 'sensor_msgs/Imu'
});

// Create a list to save the Quaternion
var xyzw = [0.0, 0.0, 0.0, 0.0];

// Subscribe to to the imu sensor
imu2Sensor.subscribe(function(message) {
    // Store the xyzw in the xyzw list. The z and x values are swapped to correct
    // the mistake in the topic
    xyzw[0] = message.orientation.z;
    xyzw[1] = message.orientation.y;
    xyzw[2] = message.orientation.x;
    xyzw[3] = message.orientation.w;
    // Show the in quaternion values in Strings of 6 decimals
    document.getElementById("x").innerHTML = "x: " + xyzw[0].toFixed(6).toString();
    document.getElementById("y").innerHTML = "y: " + xyzw[1].toFixed(6).toString();
    document.getElementById("z").innerHTML = "z: " + xyzw[2].toFixed(6).toString();
    document.getElementById("w").innerHTML = "w: " + xyzw[3].toFixed(6).toString();
});
// Create an object for the send leveling button
var sendQuatButton = document.getElementById("sendQuatButton");

// If the send button is pressed publish the inverse of the quaternion as goal to
// MoveIt!
sendQuatButton.onclick = function () {
    quaternionMessage.x = xyzw[0];
    quaternionMessage.y = xyzw[1];
    quaternionMessage.z = xyzw[2];
    quaternionMessage.w = xyzw[3];
    console.log(quaternionMessage);
    sendQuatTopic.publish(quaternionMessage);
};