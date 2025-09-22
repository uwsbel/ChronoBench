# Create the vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)  # Set contact method
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)  # Disable chassis collision
    hmmwv.SetInitPosition(ch.ChCoordsysd(chrono.ChVector3d(0, 0, 1.6), chrono.ChQuaterniond(1, 0, 0, 0)))  # Initialize vehicle position and orientation
    hmmwv.SetTireType(veh.TireModelType_TMEASY)  # Set TMEASY tire model
    hmmwv.SetTireStepSize(tire_step_size)  # Set tire simulation step size
    hmmwv.Initialize()  # Initialize the vehicle

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)  # Use primitive visualization for chassis
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)  # Use mesh visualization for suspension
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)  # Use mesh visualization for steering
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)  # Use mesh visualization for wheels
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)  # Use mesh visualization for tires

    hmmwv.GetSystem().SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)  # Set collision system to Bullet

    # Create the terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()  # Create contact material for terrain
    patch_mat.SetFriction(0.9)  # Set friction for terrain
    patch_mat.SetRestitution(0.01)  # Set restitution for terrain
    terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 50, 50)  # Add patch to terrain

    # Create the Irrlicht visualization
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Demo')  # Set window title
    vis.SetWindowSize(1280, 1024)  # Set window size
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)  # Configure chase camera
    vis.Initialize()  # Initialize visualization
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add logo to visualization
    vis.AddLightDirectional()  # Add directional light
    vis.AddSkyBox()  # Add skybox
    vis.AttachVehicle(hmmwv.GetVehicle())  # Attach vehicle to visualization

    # Create the driver system
    driver = veh.ChInteractiveDriverIRR(vis)  # Create interactive driver
    driver.Initialize()  # Initialize driver

    # Simulation loop
    time = 0  # Initialize simulation time
    time_step = 1e-3  # Define time step for simulation
    time_end = 30  # Set end time for simulation

    hmmwv.GetVehicle().EnableRealtime(True)  # Enable real-time simulation for vehicle

    while time < time_end:  # Run simulation loop while time is less than end time
        time = hmmwv.GetSystem().GetChTime()  # Update simulation time

        # Get driver inputs
        driver_inputs = driver.GetInputs()  # Get current driver inputs

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)  # Synchronize driver with current time
        terrain.Synchronize(time)  # Synchronize terrain with current time
        hmmwv.Synchronize(time, driver_inputs, terrain)  # Synchronize vehicle with driver inputs and terrain

        vis.BeginScene()  # Begin scene rendering
        vis.Render()  # Render scene
        vis.EndScene()  # End scene rendering

        # Advance simulation for one timestep for all modules
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # Spin in place for real time to catch up
        time.sleep(time_step)