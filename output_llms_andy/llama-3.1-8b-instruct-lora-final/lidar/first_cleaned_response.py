import pychrono as ch
    import pychrono.sensor as sens

    
    sys = ch.ChSystemNSC()

    
    contact_mat = ch.ChContactMaterialNSC()
    contact_mat.SetFriction(0.5)
    contact_mat.SetRestitution(0.01)

    
    mmesh_loader = ch.ChTriangleMeshLoadableWavefront()

    
    mesh = ch.ChTriangleMesh()
    mmesh_loader.LoadWavefrontMesh(mesh, "models/meshes/bulldozer/bulldozer.obj", 0.01, False)

    
    trimesh_shape = ch.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mesh)
    trimesh_shape.SetName("Bulldozer Mesh")
    trimesh_shape.SetMutable(False)

    
    body = ch.ChBodyEasy()
    body.SetPos(ch.ChVector3d(0, 0, 0))
    body.AddVisualShape(trimesh_shape)
    body.SetFixed(True)  
    body.SetContactMaterial(contact_mat)
    sys.AddBody(body)

    
    manager = sens.ChSensorManager(sys)

    
    offset_pose = ch.ChFramed(
        ch.ChVector3d(-5, 0, 1),  
        ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)),  
    )

    
    lidar = sens.ChLidarSensor(
        body,              
        update_rate,       
        offset_pose,       
        mesh,              
        360,               
        1.0,               
        90.0,              
        5,                 
        1,                 
        100.0,             
        sens.LidarBeamShape_RECTANGULAR,  
        0.01               
    )

    
    manager.AddSensor(lidar)

    
    noise_model = sens.ChNoiseModelConical()

    
    lidar.PushFilter(noise_model)

    
    filter = sens.ChFilterDIAccess()

    
    lidar.PushFilter(filter)

    
    vis_type = sens.ChVisualize()

    
    lidar.PushFilter(vis_type)

    
    save_type = sens.ChSave()

    
    lidar.PushFilter(save_type)

    
    buffer_params = sens.ChLidarBufferParameters()
    buffer_params.SetBufferSize(buffer_size)  
    buffer_params.SetBufferMode(sens.BufferMode_HISTORICAL)  

    
    lidar.SetBufferParameters(buffer_params)

    
    access_params = sens.ChLidarDataAccess()

    
    lidar.SetDataAccess(sens.ChLidarDataAccess())

    
    manager.Initialize()

    
    step_size = 1e-3

    
    while ch.ChSystem.GetChTime() < end_time:
        
        manager.Update()

        
        print("Lidar buffer data at t = ", ch.ChSystem.GetChTime())
        print(lidar.GetMostRecentBuffer().GetPointCloud())

        
        sys.DoStepDynamics(step_size)

    
    manager Finalize()