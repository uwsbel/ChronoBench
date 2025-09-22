import pychrono as ch
    import pychrono.vehicle as veh
    import pychrono.terrain as chterrain
    import pychrono.irrlicht as marea
    import math

    # -----------------
    # Create systems
    # -----------------

    # Create the FEDA vehicle and set parameters.
    feda = veh.FEDA()
    feda.SetContactMethod(ch.ChContactMethod_NSC)
    feda.SetChassisFixed(False)
    feda.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.1), ch.Quat()))
    feda.SetTireType(veh.TireModelType_TMEASY)
    feda.SetTireStepSize(1e-3)
    feda.SetChassisCollisionType(veh.CollisionType_NONE)
    feda.SetSuspensionGeometry(2.25, 0.4, 1.65, 2.25, 0.5)
    feda.SetTireRollingRadius(0.356)
    feda.SetInitPositionFWD()
    feda.SetEngineType(veh.EngineModelType_SHAFTS)
    feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    feda.SetDrivelineType(veh.DrivelineTypeAWD)
    feda.SetBrakeType(veh.BrakeType_SIMPLE)
    feda.SetMaxMotorVoltage(250)
    feda.SetTireType(veh.TireModelType_TMEASY)
    feda.SetTireStepSize(1e-3)
    feda.SetInitPosition()

    # Create the terrain
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain = chterrain.RigidTerrain(hmmwv.GetSystem())
    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 50.0, 70.0)
    patch.SetTexture(ch.GetChronoDataFile("terrain/textures/tile4.jpg"), 70, 70)
    terrain.SetGraphPlotEnabled(False)
    terrain.SetGraphWheelContactsEnabled(False)
    terrain.SetColor(0.6, 0.6, 0.6)

    # Create the vehicle Irrlicht interface
    vis = marea.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('FEDA vehicle')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(30, 10)
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(feda.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.06)
    driver.SetThrottleDelta(0.080)
    driver.SetBrakingDelta(0.080)
    driver.Initialize()

    # ------------------
    # Simulation loops
    # ------------------

    # Output vehicle mass
    print( "VEHICLE MASS: ", feda.GetVehicle().GetMass())

    # Start simulation loop
    time = 0
    time_end = 30.0
    time_step = 1e-3
    time_step_sim = 1e-3
    time_time = 0
    time_time_end = 30

    while time < time_end:
        time = veh.ChSystem.GetChTime()

        # Set driver inputs
        driver_inputs = driver.GetInputs()
        feda.SetDriverInputs(driver_inputs)

        # Draw scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Advance simulation for one timestep
        feda.GetVehicle().Advance(time_step_sim)
        terrain.Advance(time_step)

        # Increment simulation and real time
        time_time += time_step
        veh.ChSystem.GetChTime().Set(time_time)

    veh.ChSystem.GetChTime().Set(time_end)
    vis.Render()
    while vis.Run() :
        time = veh.ChSystem.GetChTime()
        time_time = time.GetAsDouble()
        if (time_time < time_time_end) :
            driver_inputs = driver.GetInputs()
            feda.SetDriverInputs(driver_inputs)
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            feda.GetVehicle().Advance(time_step_sim)
            terrain.Advance(time_step)
    vis.Render()