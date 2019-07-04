  // create the leveled goal sender ros topic with message type point
  var sendPosition = new ROSLIB.Topic({
    ros : ros,
    name : '/web_leveled_goal',
    messageType : 'geometry_msgs/Point'
  })

  // clarify how message looks like
  var xyz = new ROSLIB.Message ({
    x: 0.0,
    y: 0.0,
    z: 1.3229
  })

  // track mouse position on mousemove
  var mousePosition;
  // track state of mousedown and up
  var isMouseDown;
  // track for moving
  var mouseRelease;

  var calcX, calcY, calcZ;
  // reference to the canvas element
  var c = document.getElementById("firstCanvas");
  var c2 = document.getElementById("secondCanvas");
  // reference to 2d context
  var ctx = c.getContext("2d");
  var ctx2 = c2.getContext("2d");

  // redraw if window size changes
  document.getElementsByTagName("BODY")[0].onresize = function() {
    getElementSize();
    radius = c.width / 2.2;
    circle.r = c.height / 20;
    draw();
  };

  // check if the web application is being ran on a mobile device
   function isMobileDevice() {
    return (typeof window.orientation !== "undefined") || (navigator.userAgent.indexOf('IEMobile') !== -1);
  };

  // change the size of the two canvasses to fit in the right bar
  function getElementSize() {
    if (window.innerWidth < window.innerHeight) {
      c.width = window.innerWidth * 0.33;
      c.height = c.width;
      c2.width = c.width;
      c2.height = c.width;
    }
    else {
      c.height = window.innerHeight * 0.33;
      c.width = c.height;
      c2.width = c.width;
      c2.height = c.height * 0.8;
    };
    if (window.innerWidth < 1100) {
      c.width = window.innerWidth * 0.23;
      c.height = c.width;
      c2.width = c.width;
      c2.height = c.width;
    }

    if(isMobileDevice())
    { c.width = window.innerWidth * 0.16 ;
      c.height = c.width;
      c2.width = c.width;
      c2.height = c.width;

    }
  };
  // run on start up
  getElementSize();

  // add event listeners for mouse clicks and touches on the first canvas
  c.addEventListener('mousemove', change, false);
  c.addEventListener('mousedown', startMove, false);
  document.addEventListener('mouseup', stopMove, false);
  c.addEventListener("touchmove", change)
  c.addEventListener("touchstart", startMove)
  c.addEventListener("touchend", stopMove)

  // create a blue circle object
  var circle = new Circle(c.height/2, c.height/2, 0.15*c2.height, 0,
     c.height/20, "#0099cc", "black");
  var radius = c.width / 2.2;
  // main draw method
  function draw() {
    var fontsize = c.width/12;
    // clear canvas
    ctx.clearRect(0, 0, c.width, c.height);
    // draw the circle
    ctx.beginPath();
    ctx.arc(c.width/2, c.height/2, radius, 0, 2 * Math.PI);
    ctx.stroke();
    // clear second canvas
    ctx2.clearRect(0, 0, c2.width, c2.height);
    // draw the side views
    sideView();
    // draw the circle 
    circle.draw();
    // find and draw the coordinates on the canvas
    showCoords();
    var fontsize = c.width/12;
    ctx.font = fontsize.toString() + "px Arial";
    ctx.fillStyle = "black";
    ctx.fillText("xy view",0,fontsize);
    ctx2.font = fontsize.toString() + "px Arial";
    ctx2.fillStyle = "black";
    ctx2.textAlign = "center";
    ctx2.fillText("xz view",c2.width/2,fontsize*1.5);
    ctx2.fillText("yz view",c2.width/2,fontsize*1.5 + c2.height/2);
  };

  // draw the xz and yz viewing arcs
  function sideView() {
    ctx2.beginPath();
    ctx2.lineWidth = 1;
    ctx2.strokeStyle = "black";
    var xx = 0, x0 = 0, zz, z0 =c2.height / 2;
    for (var j=0; j<2; j++) {
        for (var i=-1; i<=1.05; i+=0.05) {
            zz = (Math.pow(i,2) * 0.266 - 0.27)*c2.height;
            if (i==-1) {
                ctx2.moveTo(x0+xx, z0+zz);
            }
            else {
                ctx2.lineTo(x0+xx, z0+zz);
            }
            xx+=0.05*c2.width/2;
        }
        ctx2.moveTo(x0, z0);
        ctx2.lineTo(x0+c2.width, z0);
        xx = 0, z0 = c2.height;
    }
    ctx2.stroke();
  }

  // function to create the Circle object
  function Circle(x, y, z, theta, r, fill, stroke) {
    this.startingAngle = 0;
    this.endAngle = 2 * Math.PI;
    this.x = x;
    this.y = y;
    this.z = z;
    this.theta = theta;
    this.r = r;
    this.fill = fill;
    this.stroke = stroke;
    // if Circle.draw is called it draws the circle and starts the function drawOtherCircles
    this.draw = function () {
        ctx.beginPath();
        ctx.arc(this.x, this.y, this.r, this.startingAngle, this.endAngle);
        ctx.fillStyle = this.fill;
        ctx.lineWidth = 1;
        ctx.fill();
        ctx.strokeStyle = this.stroke;
        ctx.stroke();
        drawOtherCircles();
    }
  }
 // the change fuction starts when the cursor or finger is moved over the canvas
  function change(){
    // it checks the mouse was pressed
    if (isMouseDown) {
        // the it finds the position of the cursor/finger on the canvas
        findMousePosition(event)
        // the functio onCircle checks if the cursor/finger is on the small circle
        // if that is true it sets mouseRelaese to false
        onCircle()
        if ( mouseRelease == false){
            // this function moves the circle to the position of the pointer
            movePointer()
        }
    }
}

// set isMouseDown to true on click or touchstart
function startMove() {
    isMouseDown = true;
}

// set stop checking for mouse position and moving control circle on mouseup or touchend
function stopMove() {
    isMouseDown = false;
    mouseRelease = true;
}

// stores the position of the pointer to x and y coordinates. It checks the type of event
// to figure out if it was a touch or mouse click.
function findMousePosition(e) {
    if(e.type == 'touchstart' || e.type == 'touchmove' || e.type == 'touchend'){
      mousePosition = {
        // find the position of the first finger on the page and substract the origin of the canvas
        x: event.touches[0].pageX - c.offsetLeft,
        y: event.touches[0].pageY - c.offsetTop
      }
    }
    else if (e.type == 'mousedown' || e.type == 'mouseup' || e.type == 'mousemove') {
      mousePosition = {
        // find the offset from the origin of the canvas
        x: event.offsetX,
        y: event.offsetY
      }
    }  
}

// function for checking if the pointer is on the circle by checking the difference of the positions 
// of the pointer and the circle. If the difference is smaller then the radius of the circle, then 
// mouseRelease is false.
function onCircle() {
    var onX = circle.x - mousePosition.x;
    var onY = circle.y - mousePosition.y;  
    if (Math.abs(onX) <= circle.r) {
        if (Math.abs(onY) <= circle.r) {
            
            mouseRelease = false;
        }
    }
    
}

// first it checks if the mouse position is within the big circle. afterwards it sets the new position
// of the circle for x and y and draws again. Also calulates the new position for the z axis with 
// the angle the middle shaft. This z coordinate is used in the side views
function movePointer() {
    if (Math.pow(mousePosition.x-c2.width/2,2) + Math.pow(mousePosition.y-c2.width/2,2) <= Math.pow(radius, 2)) {
      circle.x = mousePosition.x;
      circle.y = mousePosition.y;
      circle.theta = Math.asin(Math.sqrt(Math.pow(Math.abs(circle.x-c.width/2)/radius*0.162325, 2) + Math.pow(Math.abs(circle.y-c.width/2)/radius*0.162325, 2))/0.32465)
      circle.z = (Math.cos(circle.theta) * 0.32465 - 0.281155) / 0.162325 * radius
      draw();
      
    }
    
}
// calculate the real positions from the pixel value and stores the in a string to show on screen.
function showCoords() {
    calcX = Math.round(((circle.x-c.width/2)/radius)*0.162325*10000)/10000;
    calcY = Math.round(((circle.y-c.width/2)/radius)*0.162325*10000)/-10000;
    calcZ = Math.round((Math.cos(circle.theta)*0.32465+0.42065)*10000)/10000;
    var coor = "x: " + (calcX).toString() + "m, y: " + (calcY).toString() + "m, z: " + (calcZ).toString() + "m";
    document.getElementById("coordinates").innerHTML = coor
}

// draw side views of the circle with drawing a blue line.
function drawOtherCircles() {
    ctx2.beginPath();
    ctx2.moveTo(circle.x-circle.r*0.8,c2.height/2-circle.z*1.8);
    ctx2.lineTo(circle.x+circle.r*0.8,c2.height/2-circle.z*1.8);
    ctx2.moveTo(circle.y-circle.r*0.8,c2.height-circle.z*1.8);
    ctx2.lineTo(circle.y+circle.r*0.8,c2.height-circle.z*1.8);
    ctx2.lineWidth = 4;
    ctx2.strokeStyle = "#0099cc";
    ctx2.stroke();
}

// creates an element of the send button
var secondButton = document.getElementById('buttonSend2D')

// if clicked, publisch the the xyz to the position publisher
secondButton.onclick = function(event) {
    xyz.x = calcX;
    xyz.y = calcY;
    xyz.z = calcZ;
    sendPosition.publish(xyz)
}
// draw the view on page load
  draw();