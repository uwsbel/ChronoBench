init = chronoirr.ChInitIrrlicht()
    init.SetWindowSize(1280, 1024)
    init.SetWindowTitle('HMMWV Demo')
    init.Initialize()
    init.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    init.AddSkyBox()
    init.AddCamera(chrono.ChVector3d(0, 0.6, -1), chrono.ChVector3d(0, 0, -1))
    init.AddTypicalLights()
    init.AddLightWithShadow(chrono.ChVector3d(1.5, -2.0, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireStepSize(1e-3)
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetInitFwdVel(0.0)

    hmmwv.Initialize()

    
    sys.Add(hmmwv.GetVehicle())

    
    patch_mat = terrain.SCMMaterial()
    patch_mat.SetSoilParameters(2e6,   
                                 0,     
                                 1.1,   
                                 0,     
                                 30,    
                                 0.01,  
                                 2e8,   
                                 3e4    
                                 )
    patch_mat.SetPlotType(terrain.PLOT_SINKAGE, 0, 0.01)
    patch = terrain.SCMPatch(sys, patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(-10, 0, 0), chrono.QUNIT), 50, 50)
    patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 50, 50)
    patch.Initialize()

    
    terrain.AddPatch(patch)

    
    driver = veh.ChDriverIRR(init.GetDevice())
    driver.Initialize()

    
    driver.SetSteeringDelta(0.7)
    driver.SetThrottleDelta(0.3)
    driver.SetBrakingDelta(0.1)

    
    time_step = 1e-3

    
    time = 0
    while init.Run():
        time += time_step

        
        driver_inputs = driver.GetInputs()

        
        hmmwv.SetDriverInputs(driver_inputs)

        
        hmmwv.Update()

        
        terrain.Update(time, time_step)

        
        init.BeginScene()
        init.Render()
        init.EndScene()

        
        sys.DoStepDynamics(time_step)