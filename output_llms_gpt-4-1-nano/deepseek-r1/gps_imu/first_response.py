import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    # Set data path
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    # Create Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # Create and initialize HMMWV vehicle
    vehicle = veh.HMMWV_Full(system)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)

    # Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
    terrain.Initialize()

    # Create Irrlicht visualization
    app = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(), "HMMWV Simulation")
    app.AddTypicalLights()
    app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
    app.SetTimestep(0.01)
    app.AssetBindAll()
    app.AssetUpdateAll()

    # Create interactive driver
    driver = veh.ChIrrGuiDriver(app)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    # Create sensor manager
    sensor_manager = sens.ChSensorManager(system)
    sensor_manager.SetKeyframeSizeFromTimeStep(0.01, 1)

    # Add IMU sensor to chassis
    imu_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0))
    imu = sens.ChIMUSensor(
        vehicle.GetChassisBody(),   # parent body
        100,                        # update rate [Hz]
        imu_offset,                 # offset pose
        sens.NoiseNormal(0.1, 0.2)  # noise model
    )
    imu.SetName("IMU")
    imu.PushFilter(sens.ChFilterIMUAccess())
    sensor_manager.AddSensor(imu)

    # Add GPS sensor to chassis
    gps_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0))
    gps = sens.ChGPSSensor(
        vehicle.GetChassisBody(),   # parent body
        10,                         # update rate [Hz]
        gps_offset,                 # offset pose
        chrono.ChVectorD(0, 0, 0),  # reference location (long, lat, alt)
        sens.NoiseNormal(0.1, 0.1)  # noise model
    )
    gps.SetName("GPS")
    gps.PushFilter(sens.ChFilterGPSAccess())
    sensor_manager.AddSensor(gps)

    # Simulation loop
    step_size = 0.01
    while app.GetDevice().run():
        time = system.GetChTime()
        
        # Render scene
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()
        
        # Collect output data
        driver_inputs = driver.GetInputs()
        
        # Update modules
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        app.Synchronize("", driver_inputs)
        
        # Advance simulation
        driver.Advance(step_size)
        app.Advance(step_size)
        vehicle.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        # Update sensors
        sensor_manager.Update()
        
        # Process and display sensor data
        if imu.GetMostRecentIMUBuffer().HasData():
            imu_data = imu.GetMostRecentIMUBuffer()
            print(f"IMU - Accel: {imu_data.Acceleration} | Gyro: {imu_data.Angsy}")
            
        if gps.GetMostRecentGPSBuffer().HasData():
            gps_data = gps.GetMostRecentGPSBuffer()
            print(f"GPS - Position: {gps_data.Position} | Altitude: {gps_data.Altitude}")
        
        # Output vehicle mass
        print(f"Vehicle mass: {vehicle.GetVehicle().GetMass()} kg")

if __name__ == "__main__":
    main()