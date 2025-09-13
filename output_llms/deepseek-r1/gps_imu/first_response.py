import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    # ===== 1. Initialize Environment =====
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(100)
    system.SetGravity(chrono.ChVectorD(0, 0, -9.81))

    # ===== 2. Create Terrain =====
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # ===== 3. Create HMMWV Vehicle =====
    vehicle = veh.HMMWV_Full(system)
    vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

    # ===== 4. Configure Driver System =====
    driver = veh.ChPathFollowerDriver(vehicle, veh.SimplePath())
    driver.GetSteeringController().SetGains(0.5, 0, 0)
    driver.GetSpeedController().SetGains(0.4, 0, 0)
    
    # Create straight path
    path = veh.StraightLinePath(chrono.ChVectorD(0, 0, 0.2), chrono.ChVectorD(100, 0, 0.2), 50)
    driver.SetPath(path)
    driver.Initialize()

    # ===== 5. Create Sensors =====
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorD(10,10,40), chrono.ChColor(1,1,1), 1000)
    
    # IMU Sensor attached to chassis
    imu_offset = chrono.ChVectorD(0, 0, 1.0)
    imu_rot = chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0))
    imu = sens.ChIMUSensor(
        vehicle.GetChassisBody(),  # parent body
        50,                        # update rate [Hz]
        chrono.ChFrameD(imu_offset, imu_rot)  # offset pose
    )
    imu.SetName("Vehicle IMU")
    imu.PushFilter(sens.ChFilterAccel())
    imu.PushFilter(sens.ChFilterGyro())
    imu.PushFilter(sens.ChFilterIMUUpdate())
    manager.AddSensor(imu)

    # GPS Sensor attached to chassis
    gps_offset = chrono.ChVectorD(-1.5, 0, 0.5)
    gps = sens.ChGPSSensor(
        vehicle.GetChassisBody(),
        10,  # update rate [Hz]
        chrono.ChFrameD(gps_offset, chrono.QUNIT),
        chrono.ChVectorD(0, 0, 0),  # reference location (world origin)
        0,    # GPS reference time
        1e-3  # noise stdev
    )
    gps.SetName("Vehicle GPS")
    gps.PushFilter(sens.ChFilterGPSUpdate())
    manager.AddSensor(gps)

    # ===== 6. Set Up Visualization =====
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV Simulation with Sensors")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(8, 8, 2), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()

    # ===== 7. Simulation Loop =====
    step_size = 0.002
    realtime_timer = chrono.ChRealtimeStepTimer()
    vehicle.EnableRealtime(True)

    while vis.Run():
        time = system.GetChTime()

        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Update driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)

        # Update vehicle systems
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)

        # Update sensor manager
        manager.Update()

        # Process and output sensor data
        if imu.GetMostRecentBuffer():
            accel_data = imu.GetMostRecentBuffer().Accelerometer
            gyro_data = imu.GetMostRecentBuffer().Gyroscope
            print(f"IMU Accel: ({accel_data[0]:.3f}, {accel_data[1]:.3f}, {accel_data[2]:.3f}) m/s²")

        if gps.GetMostRecentBuffer():
            gps_data = gps.GetMostRecentBuffer().GPS
            print(f"GPS Position: ({gps_data[0]:.3f}, {gps_data[1]:.3f}, {gps_data[2]:.3f}) m")

        # Output vehicle mass
        print(f"Time: {time:.2f}s | Vehicle Mass: {vehicle.GetVehicleMass():.1f} kg")

        # Advance simulation
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        system.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()