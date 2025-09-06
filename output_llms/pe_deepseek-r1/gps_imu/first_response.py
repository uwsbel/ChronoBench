import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

def main():
    # ==========================================================================
    # 1. Initialize Simulation Environment
    # ==========================================================================
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    
    # ==========================================================================
    # 2. Create Vehicle System
    # ==========================================================================
    # Initialize HMMWV vehicle
    vehicle = veh.HMMWV_VehicleFull()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0), chrono.QUNIT))
    vehicle.Initialize()
    
    # Set vehicle subsystems
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    # Get vehicle chassis body
    chassis = vehicle.GetChassisBody()
    
    # Output vehicle mass information
    print(f"Vehicle mass: {vehicle.GetVehicleMass():.2f} kg")
    
    # ==========================================================================
    # 3. Create Terrain
    # ==========================================================================
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    
    # ==========================================================================
    # 4. Create Driver System
    # ==========================================================================
    driver = veh.ChInteractiveDriverIRR(system)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    # ==========================================================================
    # 5. Create Sensors
    # ==========================================================================
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(1, 1, 1), 5000)
    
    # Create IMU sensor attached to chassis
    imu_offset = chrono.ChVector3d(0, 0, 0.5)
    imu_pose = chrono.ChFramed(imu_offset, chrono.QUNIT)
    imu = sens.ChAccelerometerSensor(
        chassis,
        100,  # update rate [Hz]
        imu_pose,
    )
    imu.SetName("Vehicle IMU")
    imu.SetLag(0.0)
    imu.SetCollectionWindow(0.02)
    manager.AddSensor(imu)
    
    # Create GPS sensor attached to chassis
    gps_offset = chrono.ChVector3d(-1.5, 0, 0.7)
    gps_pose = chrono.ChFramed(gps_offset, chrono.QUNIT)
    gps = sens.ChGPSSensor(
        chassis,
        10,  # update rate [Hz]
        gps_pose,
        0,    # GPS reference longitude
        0,    # GPS reference latitude
        0,    # GPS reference altitude
        sens.GPSDatumType.WGS84
    )
    gps.SetName("Vehicle GPS")
    gps.SetLag(0.0)
    gps.SetCollectionWindow(0.1)
    manager.AddSensor(gps)
    
    # ==========================================================================
    # 6. Setup Visualization
    # ==========================================================================
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowTitle('HMMWV Vehicle with Sensors')
    vis.SetWindowSize(1280, 720)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(6, 6, 1.5))
    vis.AddTypicalLights()
    
    # Attach vehicle visualization
    vis.AssetBindAll()
    vis.AssetUpdateAll()
    
    # ==========================================================================
    # 7. Simulation Loop
    # ==========================================================================
    step_size = 0.002
    frame = 0
    
    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver.Synchronize(time)
        
        # Update vehicle systems
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        
        # Advance simulation
        system.DoStepDynamics(step_size)
        manager.Update()
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Process sensor data (example: every 10 steps)
        if frame % 10 == 0:
            # Get IMU data
            imu_data = imu.GetMostRecentBuffer()
            if imu_data.HasData():
                accel = imu_data.GetAccelerometerData()
                print(f"Time: {time:.2f}s | IMU Accel: X={accel[0]:.2f}, Y={accel[1]:.2f}, Z={accel[2]:.2f} m/s²")
            
            # Get GPS data
            gps_data = gps.GetMostRecentBuffer()
            if gps_data.HasData():
                gps_value = gps_data.GetGPSData()
                print(f"GPS Position: Lat={gps_value.Latitude:.6f}°, Lon={gps_value.Longitude:.6f}°, Alt={gps_value.Altitude:.2f}m")
        
        frame += 1

if __name__ == "__main__":
    main()