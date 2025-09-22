import pychrono.core as chrono
    import pychrono.sensor as sens
    import pychrono.irrlicht as chronoirr
    import math

    # Create the Chrono simulation system
    sys = chrono.ChSystemNSC()

    # Create a mesh loader for.obj files
    mesh_loader = chrono.ChTriangleMeshConnectedLoader()

    # Load a triangular mesh from a Wavefront.obj file
    mesh_loader.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mesh = mesh_loader.GetMesh()
    mesh.SetName("HMMWV Chassis Mesh")
    mesh.SetAutomaticNormalGeneration(True)
    mesh.BuildNormalMesh()

    # Create a visual shape from the mesh
    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    # Create a body and add the visual shape
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  # Set the position of the body
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)  # Fix the body in space
    sys.Add(mesh_body)  # Add the body to the simulation system

    # Create a sensor manager
    sens_manager = sens.ChSensorManager(sys)

    # Create a camera and add it to the sensor manager
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-5.0, 0, 1.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    cam = sens.ChCameraSensor(
        mesh_body,              # Body the camera is attached to
        update_rate,            # Camera update rate in Hz
        offset_pose,            # Offset pose of the camera
        image_width,            # Image width in pixels
        image_height,           # Image height in pixels
        fov                     # Camera's horizontal field of view in radians
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)  # Set the lag between sensing and data accessibility
    cam.SetCollectionWindow(exposure_time)  # Set the exposure time for the camera

    # Create a filter graph for post-processing the camera images
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  # Add constant normal noise
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  # Add pixel-dependent noise
    cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Camera Image"))  # Visualize the camera images

    sens_manager.AddSensor(cam)  # Add the camera sensor to the sensor manager

    # Create the Irrlicht visualization system
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)  # Attach the simulation system to the visualization
    vis.SetWindowSize(1280, 720)  # Set the window size for the visualization
    vis.SetWindowTitle('ChSensor test')  # Set the window title
    vis.Initialize()  # Initialize the visualization system
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
    vis.AddSkyBox()  # Add a skybox for better visual appearance
    vis.AddCamera(chrono.ChVector3d(0, 3, 6))  # Add a camera to the visualization
    vis.AddTypicalLights()  # Add typical lights for better visualization

    # Set the simulation step sizes
    step_size = 1e-3
    timestepper_step_size = step_size

    # Simulation loop
    orbit_radius = 5  # Radius of the camera orbit
    orbit_rate = 0.5   # Rate of the camera orbit in radians per second
    ch_time = 0.0  # Initialize simulation time
    cam.SetOffsetPose(chrono.ChFramed(
        chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
        chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
    ))  # Dynamically set the camera's position

    while vis.Run():  # Run the visualization loop
        vis.BeginScene()  # Begin the scene
        vis.Render()  # Render
print("error happened with only start ```python")