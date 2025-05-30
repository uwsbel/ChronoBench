import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    vehicle = veh.HMMWV_Full(system)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)

    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
    terrain.Initialize()

    
    app = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(), "HMMWV Simulation")
    app.AddTypicalLights()
    app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
    app.SetTimestep(0.01)
    app.AssetBindAll()
    app.AssetUpdateAll()

    
    driver = veh.ChIrrGuiDriver(app)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    
    sensor_manager = sens.ChSensorManager(system)
    sensor_manager.SetKeyframeSizeFromTimeStep(0.01, 1)

    
    imu_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0))
    imu = sens.ChIMUSensor(
        vehicle.GetChassisBody(),   
        100,                        
        imu_offset,                 
        sens.NoiseNormal(0.1, 0.2)  
    )
    imu.SetName("IMU")
    imu.PushFilter(sens.ChFilterIMUAccess())
    sensor_manager.AddSensor(imu)

    
    gps_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0))
    gps = sens.ChGPSSensor(
        vehicle.GetChassisBody(),   
        10,                         
        gps_offset,                 
        chrono.ChVectorD(0, 0, 0),  
        sens.NoiseNormal(0.1, 0.1)  
    )
    gps.SetName("GPS")
    gps.PushFilter(sens.ChFilterGPSAccess())
    sensor_manager.AddSensor(gps)

    
    step_size = 0.01
    while app.GetDevice().run():
        time = system.GetChTime()
        
        
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()
        
        
        driver_inputs = driver.GetInputs()
        
        
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        app.Synchronize("", driver_inputs)
        
        
        driver.Advance(step_size)
        app.Advance(step_size)
        vehicle.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        
        sensor_manager.Update()
        
        
        if imu.GetMostRecentIMUBuffer().HasData():
            imu_data = imu.GetMostRecentIMUBuffer()
            print(f"IMU - Accel: {imu_data.Acceleration} | Gyro: {imu_data.Angsy}")
            
        if gps.GetMostRecentGPSBuffer().HasData():
            gps_data = gps.GetMostRecentGPSBuffer()
            print(f"GPS - Position: {gps_data.Position} | Altitude: {gps_data.Altitude}")
        
        
        print(f"Vehicle mass: {vehicle.GetVehicle().GetMass()} kg")

if __name__ == "__main__":
    main()