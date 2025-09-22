hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)  
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)  
    hmmwv.SetInitPosition(ch.ChCoordsysd(chrono.ChVector3d(0, 0, 1.6), chrono.ChQuaterniond(1, 0, 0, 0)))  
    hmmwv.SetTireType(veh.TireModelType_TMEASY)  
    hmmwv.SetTireStepSize(tire_step_size)  
    hmmwv.Initialize()  

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)  
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)  
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)  
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)  
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)  

    hmmwv.GetSystem().SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)  

    
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()  
    patch_mat.SetFriction(0.9)  
    patch_mat.SetRestitution(0.01)  
    terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 50, 50)  

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Demo')  
    vis.SetWindowSize(1280, 1024)  
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)  
    vis.Initialize()  
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
    vis.AddLightDirectional()  
    vis.AddSkyBox()  
    vis.AttachVehicle(hmmwv.GetVehicle())  

    
    driver = veh.ChInteractiveDriverIRR(vis)  
    driver.Initialize()  

    
    time = 0  
    time_step = 1e-3  
    time_end = 30  

    hmmwv.GetVehicle().EnableRealtime(True)  

    while time < time_end:  
        time = hmmwv.GetSystem().GetChTime()  

        
        driver_inputs = driver.GetInputs()  

        
        driver.Synchronize(time)  
        terrain.Synchronize(time)  
        hmmwv.Synchronize(time, driver_inputs, terrain)  

        vis.BeginScene()  
        vis.Render()  
        vis.EndScene()  

        
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        
        time.sleep(time_step)