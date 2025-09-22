# Create the Chrono physical system and mesh loader
    sys = chrono.ChSystemSMC()
    mesh_loader = chrono.ChTriangleMeshConnectedLoader()

    # Load the triangular mesh from the Wavefront.obj file
    mesh_loader.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mesh = mesh_loader.GetMesh()

    # Scale the mesh uniformly by a factor of 2
    mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

    # Create a visual shape from the mesh
    visual_shape = chrono.ChVisualShapeTriangleMesh()
    visual_shape.SetMesh(mesh)
    visual_shape.SetName("HMMWV Chassis Mesh")
    visual_shape.SetMutable(False)  # Set the mesh to be immutable

    # Create a body for the mesh and add the visual shape
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  # Set the position of the body
    mesh_body.AddVisualShape(visual_shape)  # Add the visual shape to the body
    mesh_body.SetFixed(True)  # Fix the body in space
    sys.Add(mesh_body)  # Add the body to the physical system

    # Create a sensor manager for the simulation
    sensor_manager = chrono.ChSensorManager(sys)

    # Create a lidar sensor and configure its properties
    lidar = chrono.ChLidarSensor(mesh_body, update_rate, \
        chrono.ChVector3d(0, 0.5, 0), chrono.ChVector3d(0, 0, 0), \
        horizontal_samples, vertical_samples, horizontal_fov, vertical_fov, \
        100.0, 1000.0, return_mode, depth_image_mode, \
        divergence_angle, divergence_angle, divergence_angle, \
        receiver_field_of_view, receiver_field_of_view, receiver_field_of_view, \
        lidar_name)

    lidar.SetName(lidar_name)  # Set the name of the lidar sensor
    lidar.SetLag(lag)  # Set the lag time for the lidar
    lidar.SetCollectionWindow(collection_time)  # Set the collection window for the lidar
    lidar.SetMaxDepth(max_depth)  # Set the maximum depth range for the lidar
    lidar.SetMinDepth(min_depth)  # Set the minimum depth range for the lidar
    lidar.SetReturnMode(return_mode)  # Set the return mode for the lidar
    lidar.SetDepthImageMode(depth_image_mode)  # Set the depth image mode for the lidar
    lidar.SetDivergenceAngle(divergence_angle)  # Set the divergence angle for the lidar
    lidar.SetReceiverFieldOfView(receiver_field_of_view)  # Set the receiver field of view for the lidar

    # Add a filter graph to the lidar sensor
    if noise_model!= "NONE":
        lidar.AddFilter(chrono.ChFilterLidarNoise( \
            noise_model, noise_model, noise_model, \
            noise_std, noise_std, noise_std, \
            temporal_noise_profile, temporal_noise_profile, temporal_noise_profile))

    if vis:
        lidar.AddFilter(chrono.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Depth Data"))

    if save:
        lidar.AddFilter(chrono.ChFilterSave(horizontal_samples, vertical_samples, "lidar_data"))

    sensor_manager.AddSensor(lidar)  # Add the lidar sensor to the sensor manager

    # Initialize the simulation driver
    driver = chrono.ChInteractiveDriverIRR(sensor_manager)

    # Set the initial camera position and orientation
    driver.SetCamera(chrono.ChVector3d(0, 1, 0.5), chrono.ChVector3d(0, 0, 0.5))

    # Run the simulation loop
    while (driver.Run()):
        driver_inputs = driver.GetInputs()  # Get the current driver inputs
        lidar_data = lidar.GetMostRecentBuffer()  # Get the most recent lidar data
        if lidar_data.HasData():  # Check if the lidar data is available
            lidar_buffer = lidar_data.GetBuffer()  # Get the lidar buffer data
            print("Lidar Buffer Data:", lidar_buffer)  # Print the lidar buffer data
        sys.DoStepDynamics(step_size)  # Perform a simulation step with the specified step size
        driver.Synchronize(sys.GetChTime())  # Synchronize the driver with the simulation time