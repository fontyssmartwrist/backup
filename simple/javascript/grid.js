// Create a topic for sending joint angles.
var sendGrid = new ROSLIB.Topic({
  ros : ros,
  name : '/web_laser',
  messageType : 'geometry_msgs/Point'
})
// Setup a zero message.
var gridGoal = new ROSLIB.Message ({
  x: 0,
  y: 0,
  z: 0
})

// Create an js object for the canvas, the content and the right bar space 
var c3 = document.getElementById("gridCanvas");
var ctx3 = c3.getContext("2d");
var navspace = document.getElementById("rightbar");

// Sets dimentions of the grid to fit the window
if ( navspace.clientWidth>= navspace.clientHeight) {
    c3.width = navspace.clientHeight;
    c3.height = navspace.clientHeight;
}
else {
    c3.width = navspace.clientWidth;
    c3.height = navspace.clientWidth;
}

// Look for mouse button press or end of a touch and start function addWaypoint
c3.addEventListener('mousedown', addWaypoint, false);
c3.addEventListener("touchend", addWaypoint, false);

// Creat variables for amount of rows and columns. Step left and top is the length of each 
// column/row + 2 on the grid. The addition is for creating space around the grid.
var rows = 10, columns = 10, stepLeft = c3.width/(columns + 2), stepTop = c3.height/(rows + 2)

// Correct x,y offset
var topOffset = c3.offsetTop
var leftOffset = c3.offsetLeft

// Draws a new clear canvas and start the function to draw the grid
function drawCanvas() {
  ctx3.clearRect(0, 0, c3.width, c3.height);
  drawGrid();
}

// Draws the grid with line Width 1, black as the color and columns + 1 vertical lines and
// rows + 1 horizontal lines
function drawGrid() {
  ctx3.beginPath();
  ctx3.lineWidth = 1;
  ctx3.strokeStyle = "black";
  for (var left = 1; left <= columns+ 1; left++) {
    ctx3.moveTo(left*stepLeft, stepTop);
    ctx3.lineTo(left*stepLeft, c3.height - stepTop);
  };
  for (var down = 1; down <= rows +1 ; down++) {
    ctx3.moveTo(stepLeft, down*stepTop);
    ctx3.lineTo(c3.width - stepLeft, down*stepTop);
  };
  ctx3.stroke();
};

// Find the position of click/touch on the grid. Then it clears the canvas and checks if the position
// was on the grid. If true, then start the function colourBlock to show it was pressed.
function addWaypoint() {
  setMousePosition(event);
  drawCanvas();
  
  if (mousePos.x > stepLeft && mousePos.x < c3.width - stepLeft 
    && mousePos.y > stepTop && mousePos.y < c3.height - stepTop) {
    colourBlock();
  };
  
};

// If find the x and y coordinate of the touch on the page by substracting the coordinate of the canvas.
// It finds the mouse coordinates by x and y offset from the origin of the canvas. It stores them in 
// the MousePos object.
function setMousePosition(e) {
  if(e.type == 'touchend'){
    mousePos = {
      x: event.touches[0].pageX - leftOffset,
      y: event.touches[0].pageY - topOffset
    };  
  }
  else if (e.type == 'mousedown') {
    mousePos = {
      x: event.offsetX,
      y: event.offsetY
    };
  }; 
};

// Colours a green circle in the middle on the selected block.
function colourBlock() {
  block = {
    x: Math.floor(mousePos.x/stepLeft),
    y: Math.floor(mousePos.y/stepTop)
  };
  var fontSize = stepTop/2;
  ctx3.font = fontSize.toString() + "px Helvetica";
  ctx3.textAlign = "center";
  ctx3.fillStyle = "green";
  ctx3.fillText("•", (block.x+0.5)*stepLeft, (block.y+0.5)*stepTop+fontSize/3);
  // calculateTheta(); 
  sendGridTopic(block);
  return block;
};
/*
// Calculate the joint angles for one wrist
var cellwidth = 0.04;
var length = 1.844;

function calculateTheta () {
  var x = block.x;
  var y = block.y;
  var xreal = -(5.5-x)*cellwidth;
  var yreal = (5.5-y)*cellwidth;
  var distancefromcenter = Math.sqrt(Math.pow(xreal,2)+Math.pow(yreal,2));
  var theta2 = 2*Math.atan(distancefromcenter/length);
  var theta1;
  if (xreal > 0 && yreal >= 0) {
    theta1 = Math.atan(yreal/xreal) - ((Math.PI-theta2)/2);
  }
  else if (xreal < 0 && yreal >= 0) {
    theta1 = Math.atan(yreal/xreal) - ((Math.PI-theta2)/2) + Math.PI;
  }
  else if (xreal < 0 && yreal < 0) {
    theta1 = Math.atan(yreal/xreal) - ((Math.PI-theta2)/2) + Math.PI;
  }
  else if (xreal > 0 && yreal < 0) {
    theta1 = Math.atan(yreal/xreal) - ((Math.PI-theta2)/2);
  };
  if (theta1 > Math.PI) {
    theta1 = theta1 - 2 * Math.PI;
  };
  sendLaserTopic(theta1,theta2);
}

// Publishes the joint angles
function sendLaserTopic (theta3,theta4) {
  jointGoal.data[0] = 0
  jointGoal.data[1] = 0
  jointGoal.data[2] = theta3
  jointGoal.data[3] = theta4
  jointGoal.data[4] = 0
  sendAngles.publish(jointGoal)
}
*/
function sendGridTopic (coordinate) {
  gridGoal.x = coordinate.x-1
  gridGoal.y = coordinate.y-1
  console.log(gridGoal)
  sendGrid.publish(gridGoal)
}
drawCanvas();