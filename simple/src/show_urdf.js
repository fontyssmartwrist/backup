/**
   * Setup all visualization elements when the page is loaded.
   */
  
  function show3DJS() {
    // Connect to ROS.
    var ros = new ROSLIB.Ros({
      url : 'ws://145.93.88.207:9090'
      // url : 'ws://localhost:9090'
    });

    var viewer = new ROS3D.Viewer({
      divID : 'urdf',
      width : 1286,
      height : 1080,
      antialias : true,
      background : '#EEEEEEE',   
    });
    // Add a grid.
    viewer.addObject(new ROS3D.Grid({
      cellSize : 0.5,
      color : '#0099cc'
    }));
    
    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      angularThres : 0.01,
      transThres : 0.01,
      rate : 10.0
    });

    // Setup the URDF client.
    var urdfClient = new ROS3D.UrdfClient({
      ros : ros,
      tfClient : tfClient,
      path : 'http://145.93.88.207:8001',
      // path : 'http://localhost:8001',
      rootObject : viewer.scene,
      loader : ROS3D.COLLADA_LOADER_2
    });
  }