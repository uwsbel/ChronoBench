# Initialize the PyChrono environment and core components
    init = chronoirr.ChInitIrrlicht()
    init.SetWindowSize(1280, 1024)
    init.SetWindowTitle('HMMWV Demo')
    init.Initialize()
    init.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    init.AddSkyBox()
    init.AddCamera(chrono.ChVector3d(0, 0.6, -1), chrono.ChVector3d(0, 0, -1))
    init.AddTypicalLights()
    init.AddLightWithShadow(chrono.ChVector3d(1.5, -2.0, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Create the HMMWV vehicle and set parameters
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

    # Add the HMMWV to the simulation system
    sys.Add(hmmwv.GetVehicle())

    # Create the SCM deformable terrain patch
    patch_mat = terrain.SCMMaterial()
    patch_mat.SetSoilParameters(2e6,   # Bekker Kphi
                                 0,     # Bekker Kc
                                 1.1,   # Bekker n exponent
                                 0,     # Mohr cohesive limit (Pa)
                                 30,    # Mohr friction limit (degrees)
                                 0.01,  # Janosi shear coefficient (m)
                                 2e8,   # Elastic stiffness (Pa/m), before plastic yield
                                 3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
                                 )
    patch_mat.SetPlotType(terrain.PLOT_SINKAGE, 0, 0.01)
    patch = terrain.SCMPatch(sys, patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(-10, 0, 0), chrono.QUNIT), 50, 50)
    patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 50, 50)
    patch.Initialize()

    # Add the patch to the terrain
    terrain.AddPatch(patch)

    # Create the driver system
    driver = veh.ChDriverIRR(init.GetDevice())
    driver.Initialize()

    # Set the time response for steering and throttle keyboard inputs.
    driver.SetSteeringDelta(0.7)
    driver.SetThrottleDelta(0.3)
    driver.SetBrakingDelta(0.1)

    # Set the simulation time step
    time_step = 1e-3

    # Simulation loop
    time = 0
    while init.Run():
        time += time_step

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Set driver inputs to vehicle
        hmmwv.SetDriverInputs(driver_inputs)

        # Update vehicle dynamics
        hmmwv.Update()

        # Update SCM terrain (rendering is included in this step)
        terrain.Update(time, time_step)

        # Render the scene
        init.BeginScene()
        init.Render()
        init.EndScene()

        # Advance simulation by one time step
        sys.DoStepDynamics(time_step)