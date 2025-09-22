sys = chrono.ChSystemSMC()
    mesh_loader = chrono.ChTriangleMeshConnectedLoader()

    
    mesh_loader.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mesh = mesh_loader.GetMesh()

    
    mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

    
    visual_shape = chrono.ChVisualShapeTriangleMesh()
    visual_shape.SetMesh(mesh)
    visual_shape.SetName("HMMWV Chassis Mesh")
    visual_shape.SetMutable(False)  

    
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  
    mesh_body.AddVisualShape(visual_shape)  
    mesh_body.SetFixed(True)  
    sys.Add(mesh_body)  

    
    sensor_manager = chrono.ChSensorManager(sys)

    
    lidar = chrono.ChLidarSensor(mesh_body, update_rate, \
        chrono.ChVector3d(0, 0.5, 0), chrono.ChVector3d(0, 0, 0), \
        horizontal_samples, vertical_samples, horizontal_fov, vertical_fov, \
        100.0, 1000.0, return_mode, depth_image_mode, \
        divergence_angle, divergence_angle, divergence_angle, \
        receiver_field_of_view, receiver_field_of_view, receiver_field_of_view, \
        lidar_name)

    lidar.SetName(lidar_name)  
    lidar.SetLag(lag)  
    lidar.SetCollectionWindow(collection_time)  
    lidar.SetMaxDepth(max_depth)  
    lidar.SetMinDepth(min_depth)  
    lidar.SetReturnMode(return_mode)  
    lidar.SetDepthImageMode(depth_image_mode)  
    lidar.SetDivergenceAngle(divergence_angle)  
    lidar.SetReceiverFieldOfView(receiver_field_of_view)  

    
    if noise_model!= "NONE":
        lidar.AddFilter(chrono.ChFilterLidarNoise( \
            noise_model, noise_model, noise_model, \
            noise_std, noise_std, noise_std, \
            temporal_noise_profile, temporal_noise_profile, temporal_noise_profile))

    if vis:
        lidar.AddFilter(chrono.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Depth Data"))

    if save:
        lidar.AddFilter(chrono.ChFilterSave(horizontal_samples, vertical_samples, "lidar_data"))

    sensor_manager.AddSensor(lidar)  

    
    driver = chrono.ChInteractiveDriverIRR(sensor_manager)

    
    driver.SetCamera(chrono.ChVector3d(0, 1, 0.5), chrono.ChVector3d(0, 0, 0.5))

    
    while (driver.Run()):
        driver_inputs = driver.GetInputs()  
        lidar_data = lidar.GetMostRecentBuffer()  
        if lidar_data.HasData():  
            lidar_buffer = lidar_data.GetBuffer()  
            print("Lidar Buffer Data:", lidar_buffer)  
        sys.DoStepDynamics(step_size)  
        driver.Synchronize(sys.GetChTime())