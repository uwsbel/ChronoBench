import pychrono.core as chrono
    import pychrono.sensor as sens
    import pychrono.irrlicht as chronoirr
    import math

    
    sys = chrono.ChSystemNSC()

    
    mesh_loader = chrono.ChTriangleMeshConnectedLoader()

    
    mesh_loader.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mesh = mesh_loader.GetMesh()
    mesh.SetName("HMMWV Chassis Mesh")
    mesh.SetAutomaticNormalGeneration(True)
    mesh.BuildNormalMesh()

    
    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)  
    sys.Add(mesh_body)  

    
    sens_manager = sens.ChSensorManager(sys)

    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-5.0, 0, 1.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    cam = sens.ChCameraSensor(
        mesh_body,              
        update_rate,            
        offset_pose,            
        image_width,            
        image_height,           
        fov                     
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)  
    cam.SetCollectionWindow(exposure_time)  

    
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  
    cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Camera Image"))  

    sens_manager.AddSensor(cam)  

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)  
    vis.SetWindowSize(1280, 720)  
    vis.SetWindowTitle('ChSensor test')  
    vis.Initialize()  
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
    vis.AddSkyBox()  
    vis.AddCamera(chrono.ChVector3d(0, 3, 6))  
    vis.AddTypicalLights()  

    
    step_size = 1e-3
    timestepper_step_size = step_size

    
    orbit_radius = 5  
    orbit_rate = 0.5   
    ch_time = 0.0  
    cam.SetOffsetPose(chrono.ChFramed(
        chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
        chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
    ))  

    while vis.Run():  
        vis.BeginScene()  
        vis.Render()  
print("error happened with only start ```python")