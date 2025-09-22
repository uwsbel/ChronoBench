import pychrono as ch
    import pychrono.sensor as sens

    
    sys = ch.ChSystemNSC()

    
    mmesh_loader = ch.ChTriangleMeshLoadableWavefront()

    
    mesh = ch.ChTriangleMesh()
    mesh.LoadWavefront("models/meshes/bulldozer/bulldozer.obj", mmesh_loader, sys)

    
    contact_material = ch.ChContactMaterialNSC()
    contact_material.SetFriction(0.5)
    contact_material.SetRestitution(0.01)

    
    visual_shape = ch.ChVisualShapeTriangleMesh()
    visual_shape.SetMesh(mesh)
    visual_shape.SetMutable(False)  
    visual_shape.SetTextureScale(ch.ChVector3d(0.02, 0.02, 1.0))  

    
    body = ch.ChBodyEasyMesh()
    body.SetContactMaterial(contact_material)
    body.AddVisualShape(visual_shape)
    body.SetFixed(True)  
    body.SetPos(ch.ChVector3d(0, 0, 0))  
    sys.AddBody(body)  

    
    manager = sens.ChSensorManager(sys)

    
    offset_pose = ch.ChFramed(ch.ChVector3d(-5, 0, 1), ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))

    
    cam = sens.ChCameraSensor(
        body,              
        30,                
        24,                
        offset_pose,       
        sens.NSensorShape::Camera
    )

    
    cam.PushFilter(sens.ChFilterVisualize())
    cam.PushFilter(sens.ChFilterDIAccess())
    cam.PushFilter(sens.ChFilterRGB8Access())

    
    manager.AddSensor(cam)

    
    noise_model = sens.ChNoiseModelNone()

    
    lens_model = sens.ChLensModelPinHole()

    
    orbit_radius = 10.0
    orbit_center = ch.ChVector3d(0, 0, 0)
    orbit_rate = 0.5  

    
    time = 0
    time_end = 30.0
    time_step = 1e-3
    timeaccumulator = 0.0
    while time < time_end:
        time = time + time_step
        timeaccumulator += time_step

        
        cam.SetOffsetPose(ch.ChFramed(
            ch.ChVector3d(orbit_radius * math.cos(orbit_rate * time), orbit_radius * math.sin(orbit_rate * time), 1),
            ch.QuatFromAngleAxis(orbit_rate * time, ch.ChVector3d(0, 1, 0))
        ))

        
        cam.GetLatestImage().GetBuffer().GetRGB8Data().CopyToRGBA8Buffer()

        
        print("Camera buffer data at time: ", time, " is: ", cam.GetLatestImage().GetBuffer().GetRGB8Data().GetHostMemoryPointer())

        
        manager.Update()

        
        sys.DoStepDynamics(time_step)

    print("Done!")