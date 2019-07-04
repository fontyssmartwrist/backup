var nav1 = document.getElementById("robotStatus");
var nav2 = document.getElementById("2dControl");
var nav3 = document.getElementById("sliders");
var nav4 = document.getElementById("laserGrid")
var nav5 = document.getElementById("levelControl")
nav1.style.display = "inline";
nav2.style.display = "none";
nav3.style.display = "none";
nav4.style.display = "none";
nav5.style.display = "none";


function showStatus() {
  waitForService ("Req:[1]");
  waitForService('Laser:[0]');
  nav1.style.display = "inline";
  nav2.style.display = "none";
  nav3.style.display = "none";
  nav4.style.display = "none";
  nav5.style.display = "none";
}
  function show2DControl() {
    waitForService('Laser:[0]');
    draw();
    nav1.style.display = "none";
    nav2.style.display = "inline";
    nav3.style.display = "none";
    nav4.style.display = "none";
    nav5.style.display = "none";
  }
  function showSliders() {
    waitForService('Laser:[0]');
    nav1.style.display = "none";
    nav2.style.display = "none";
    nav3.style.display = "inline";
    nav4.style.display = "none";
    nav5.style.display = "none";
  }
  function showGrid() {
    waitForService('Laser:[1]');
    drawCanvas();
    nav1.style.display = "none";
    nav2.style.display = "none";
    nav3.style.display = "none";
    nav4.style.display = "inline";
    nav5.style.display = "none";
  }
  function showLevel() {
    waitForService('Laser:[0]');
    nav1.style.display = "none";
    nav2.style.display = "none";
    nav3.style.display = "none";
    nav4.style.display = "none";
    nav5.style.display = "inline";
  }