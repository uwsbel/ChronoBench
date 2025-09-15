#!/usr/bin/env python3
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    # 1. Initialize system
    chrono.SetChronoDataPath("path/to/chrono/data/")  # Update this path
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVectorD(0, 0, -9.81))
    
    # 2. Create terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.Initialize(chrono.ChVectorD(0, 0, 0), patch_mat, 200, 0.1)
    
    # 3. Create and initialize HMMWV
    hmmwv = veh.HMMWV_Full(system)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    hmmwv.Initialize()
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    # 4. Create driver system
    driver = veh.ChInteractiveDriverIRR(hmmwv.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    # 5. Create sensors
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 500)
    
    # IMU sensor attached to chassis
    imu_offset = chrono.ChVectorD(0, 0, 0.5)
    imu_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    imu = sens.ChAccelerometerSensor(
        hmmwv.GetChassisBody(),          # parent body
        100,                             # update rate [Hz]
        chrono.ChFrameD(imu_offset, imu_rot),  # offset pose
        sens.ChNoiseNormal(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0.001))  # noise model
    )
    imu.SetName("Chassis IMU")
    imu.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(imu)
    
    # GPS sensor attached to chassis
    gps_offset = chrono.ChVectorD(-1.5, 0, 0.5)
    gps = sens.ChGPSSensor(
        hmmwv.GetChassisBody(),
        10,                              # update rate [Hz]
        chrono.ChFrameD(gps_offset, imu_rot),
        sens.ChNoiseNormal(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0.001))
    )
    gps.SetName("Chassis GPS")
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)
    
    # 6. Create Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV with Sensors")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(8, 8, 1), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    hmmwv.GetVehicle().AddVisualizationAssets(irr)
    
    # 7. Simulation loop
    step_size = 0.002
    frame = 0
    
    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        
        # Update vehicle
        hmmwv.Synchronize(time, driver_inputs, terrain)
        
        # Advance system
        manager.Update()
        terrain.Synchronize(time)
        hmmwv.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Process sensor data and output info
        if frame % 50 == 0:
            # Output vehicle mass
            mass = hmmwv.GetVehicle().GetMass()
            print(f"Time: {time:.2f}s | Vehicle Mass: {mass:.2f} kg")
            
            # Output IMU data
            imu_data = imu.GetMostRecentBuffer()
            if imu_data.HasData():
                accel = imu_data.GetAccelData()
                print(f"IMU Acceleration: X={accel[0]:.4f}, Y={accel[1]:.4f}, Z={accel[2]:.4f} m/s²")
            
            # Output GPS data
            gps_data = gps.GetMostRecentBuffer()
            if gps_data.HasData():
                pos = gps_data.GetGPSData()
                print(f"GPS Position: Lat={pos.Latitude:.6f}, Lon={pos.Longitude:.6f}, Alt={pos.Altitude:.2f} m")
        
        frame += 1

if __name__ == "__main__":
    main()