/* This script does the joint control*/
  
  // Create a topic for sending joint angles.
  var sendAngles = new ROSLIB.Topic({
    ros : ros,
    name : '/web_goal',
    messageType : 'std_msgs/Float32MultiArray'
  })
  // Setup a zero message.
  var jointGoal = new ROSLIB.Message ({
    data: [0.0, 0.0, 0.0, 0.0, 0.0]
  })

  // Create objects of the elements.
  var rangeslider1 = document.getElementById("sliderRange") 
  var rangeslider2 = document.getElementById("sliderRange2")
  var rangeslider3 = document.getElementById("sliderRange3") 
  // var rangeslider4 = document.getElementById("sliderRange4") 
  // var rangeslider5 = document.getElementById("sliderRange5") 
  var output1 = document.getElementById("demo1")
  var output2 = document.getElementById("demo2")
  var output3 = document.getElementById("demo3")
  // var output4 = document.getElementById("demo4")
  // var output5 = document.getElementById("demo5")
  var firstButton = document.getElementById('buttonHome')
  var secondButton = document.getElementById('buttonSend')
  var thirdButton = document.getElementById('buttonCurrent')
  var fourthButton = document.getElementById('buttonRandom')
  var jointValue1, jointValue2, jointValue3, jointValue4, jointValue5

  // Set up all the joint values to rangeslider values on start up.
  changeValue1(rangeslider1.value)
  changeValue2(rangeslider2.value)
  changeValue3(rangeslider3.value)
  // changeValue4(rangeslider4.value)
  // changeValue5(rangeslider5.value)

  // Change the displayed joint value if the the rangeslider gets an input.
  rangeslider1.oninput = function() {
    changeValue1(rangeslider1.value)
  } 
  rangeslider2.oninput = function() {
    changeValue2(rangeslider2.value)
  } 
  rangeslider3.oninput = function() {
    changeValue3(rangeslider3.value)
  } 
  // rangeslider4.oninput = function() {
  //   changeValue4(rangeslider4.value)
  // } 
  // rangeslider5.oninput = function() {
  //   changeValue5(rangeslider5.value)
  // } 

  
  // The home button sets all the sliders to the 0 zero position.
  firstButton.onclick = function(event) {
    var home = [0, 0, 0, 0, 0]
    setSliders(home);
  }
  // If the send button is clicked the values are send.
  secondButton.onclick = function(event) {
    sendTopic();
  }
  // Load all the joint values in the joint goal message and publish it.
  function sendTopic () {
    jointGoal.data[0] = jointValue1;
    jointGoal.data[1] = jointValue2;
    jointGoal.data[2] = jointValue3;
    jointGoal.data[3] = 0.0;
    jointGoal.data[4] = 0.0;
    sendAngles.publish(jointGoal);
  }
  // Does a service request to get the current joint values
  thirdButton.onclick = function(event) {
    waitForService('Current');
  }
  // Set the rangesliders to given angle values and change the text
  function setSliders (joints) {
    rangeslider1.value = joints[0];
    rangeslider2.value = joints[1];
    rangeslider3.value = joints[2];
    // rangeslider4.value = joints[3];
    // rangeslider5.value = joints[4];
    changeValue1(rangeslider1.value);
    changeValue2(rangeslider2.value);
    changeValue3(rangeslider3.value);
    // changeValue4(rangeslider4.value);
    // changeValue5(rangeslider5.value);
  }
 
  // Set a random value for each of the range sliders
  fourthButton.onclick = function(event) {
    rangeslider1.value = Math.round(Math.random() * (100)) - 50
    changeValue1(rangeslider1.value)
    rangeslider2.value = Math.round(Math.random() * (100)) - 50
    changeValue2(rangeslider2.value)
    rangeslider3.value = Math.round(Math.random() * (100)) - 50
    changeValue3(rangeslider3.value)
    // rangeslider4.value = Math.round(Math.random() * (100)) - 50
    // changeValue4(rangeslider4.value)
    // rangeslider5.value = Math.round(Math.random() * (100)) - 50
    // changeValue5(rangeslider5.value)
  }
  // Functions for going from rangeslider values to joint angles 
  // and display them on the page.
  function changeValue1 (changeto) {
    jointValue1 = (changeto/50)*Math.PI
    output1.innerHTML = jointValue1.toFixed(6)
  }
  function changeValue2 (changeto) {
    jointValue2 = (changeto/50)*Math.PI
    output2.innerHTML = jointValue2.toFixed(6)
  }
  function changeValue3 (changeto) {
    jointValue3 = (changeto/50)*Math.PI
    output3.innerHTML = jointValue3.toFixed(6)
  }
  // function changeValue4 (changeto) {
  //   jointValue4 = (changeto/50)*Math.PI
  //   output4.innerHTML = jointValue4.toFixed(6)
  // }
  // function changeValue5 (changeto) {
  //   jointValue5 = (changeto/50)*Math.PI
  //   output5.innerHTML = jointValue5.toFixed(6)
  // }