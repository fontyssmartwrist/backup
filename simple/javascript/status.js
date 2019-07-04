// First, we create a Service client with details of the service's name and service type.
var robotStatusClient = new ROSLIB.Service({
  ros : ros,
  name : 'web_status_speed_srv',
  serviceType : 'smart_movement/RequestStatus'
});

// Then we create a Service Request. 
var servicerequest = new ROSLIB.ServiceRequest({
  statusreq: "req:[1]"
});

// To make the Service executable we start with a function. The argument is the String
// that needs be send to ROS.
function waitForService (servicemessage) {
  // Put the String in the Request
  servicerequest.statusreq = servicemessage;
  console.log(servicerequest);
  // Now it's time to call the service, with the result as the responce from the Server.
  robotStatusClient.callService(servicerequest, function(result) {
    console.log(result.statusrep)
    // Look if the speedmode was send from CODESYS and displays it in the status window.
    if (result.statusrep.includes("Speed")) {
      var speedpos = result.statusrep.indexOf("Speed");
      var speedsetting = result.statusrep[speedpos+7];
      var speedname;
      if (speedsetting == "6") {
        speedname = "Swift"
      }
      else if (speedsetting == "3") {
        speedname = "Moderate"
      }
      else if (speedsetting == "1") {
        speedname = "Debug"
      }
      else {
        speedname = "Error"
      }        
      document.getElementById("speedMode").innerHTML = "Speed mode: " + speedname;
    };
    // Look if the status of the motors was send from CODESYS and displays it in the
    // status window.
    if (result.statusrep.includes("Motor")) {
      var motorpos = result.statusrep.indexOf("Motor");
      var motor = result.statusrep.substr(motorpos+6,9);
      document.getElementById("motorSetting").innerHTML = "Motors: " + motor;
    };
    // Look if the emergency button state was send from CODESYS and displays it in the 
    // status window. Als says "Pushed!"" if an emergency button error occurs.
    if (result.statusrep.includes("Emergency")) {
      var emergencypos = result.statusrep.indexOf("Emergency");
      var emergency;
      if (result.statusrep[emergencypos+11] == "1" 
       || result.statusrep[emergencypos+13] == "1") {
        emergency = "Pushed!";
      }
      else {
        emergency = "Operational";
      }
      document.getElementById("emergency").innerHTML = "Emergency button: " + emergency;
    };
    // Look if the Smart Wrist status was send from CODESYS and displays it in the status 
    // window. Also shows that the connection to CODESYS was established.
    if (result.statusrep.includes("Start")) {
      var startpos = result.statusrep.indexOf("Start");
      var start
      if (result.statusrep[startpos+7] == "1") {
        start = "Ready for new command";
      }
      else {
        start = "Halted";
      }
      document.getElementById("systemStart").innerHTML = "Movement: " + start;
      document.getElementById("codesysCon").innerHTML = "CODESYS connection: Established";
    };

    if (result.statusrep.includes("Enc")) {
      var encpos = result.statusrep.indexOf("Enc");
      var jointstart = result.statusrep.indexOf("[",encpos);
      var jointend = result.statusrep.indexOf("]",encpos);
      var joints = result.statusrep.substr(jointstart, jointend - jointstart + 1);
      document.getElementById("jointAngles").innerHTML = "Joint angles: " + joints;

    }

    if (result.statusrep.includes("Joint")) {
      var joint1start = result.statusrep.indexOf("[");
      var joint2start = result.statusrep.indexOf(",");
      var joint3start = result.statusrep.indexOf(",", joint2start+1);
      var joint4start = result.statusrep.indexOf(",", joint3start+1);
      var joint5start = result.statusrep.indexOf(",", joint4start+1);
      var joint5end = result.statusrep.indexOf("]");
      var joint1 = result.statusrep.substr(joint1start+1,joint2start - (joint1start + 1));
      var joint2 = result.statusrep.substr(joint2start+1,joint3start - (joint2start + 2));
      var joint3 = result.statusrep.substr(joint3start+1,joint4start - (joint3start + 2));
      var joint4 = result.statusrep.substr(joint4start+1,joint5start - (joint4start + 2));
      var joint5 = result.statusrep.substr(joint5start+1,joint5end - (joint5start + 1));
      joint1 = Math.round(parseFloat(joint1)* (50) / Math.PI);
      joint2 = Math.round(parseFloat(joint2)* (50) / Math.PI);
      joint3 = Math.round(parseFloat(joint3)* (50) / Math.PI);
      joint4 = Math.round(parseFloat(joint4)* (50) / Math.PI);
      joint5 = Math.round(parseFloat(joint5)* (50) / Math.PI);
      var joints = [joint1, joint2, joint3, joint4, joint5];
      setSliders(joints); 
    };
  });
}
// Does a request to the server on page load.
waitForService ("Req:[1]");