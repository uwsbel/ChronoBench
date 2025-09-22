import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import numpy as np
import math

def main():
    
    chrono.SetChronoDataPath("C:/ProgramData/Anaconda3/envs/pychrono/Library/data/")
    veh.SetDataPath("C:/ProgramData/Anaconda3/envs/pychrono/Library/data/vehicle/")
    
    
    print("Creating vehicle system...")
    
    
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.6), chrono.ChQuaternionD(1, 0, 0, 0)))
    vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()
    
    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    vehicle_mass = vehicle.GetVehicle().GetMass()
    print(f"Vehicle mass: {vehicle_mass:.2f} kg")
    
    
    print("Creating terrain...")
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
                           200, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    
    
    print("Creating driver system...")
    driver = veh.ChInteractiveDriverIRR()
    
    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(1.0 / steering_time)
    driver.SetThrottleDelta(1.0 / throttle_time)
    driver.SetBrakingDelta(1.0 / braking_time)
    
    
    print("Creating Irrlicht visualization...")
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())
    
    
    driver.Initialize()
    
    
    print("Creating sensor manager...")
    manager = sens.ChSensorManager(vehicle.GetSystem())
    
    
    manager.scene.AddPointLight(chrono.ChVectorF(0, 0, 100), chrono.ChColor(1, 1, 1), 5000.0)
    manager.scene.SetAmbientLight(chrono.ChColor(0.1, 0.1, 0.1))
    manager.scene.SetSceneEpsilon(1e-3)
    
    
    chassis_body = vehicle.GetVehicle().GetChassisBody()
    
    
    print("Creating IMU sensor...")
    imu_offset_pos = chrono.ChVectorD(0, 0, 0)  
    imu_offset_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
    
    imu = sens.ChIMUSensor(
        chassis_body,
        100,  
        chrono.ChFrameD(imu_offset_pos, imu_offset_rot)
    )
    
    
    imu.SetNoiseNone()  
    
    
    manager.AddSensor(imu)
    
    
    print("Creating GPS sensor...")
    gps_offset_pos = chrono.ChVectorD(0, 0, 0.5)  
    gps_offset_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
    
    gps = sens.ChGPSSensor(
        chassis_body,
        10,  
        chrono.ChFrameD(gps_offset_pos, gps_offset_rot),
        chrono.ChVectorD(0, 0, 0)  
    )
    
    
    gps.SetNoiseNone()  
    
    
    manager.AddSensor(gps)
    
    
    step_size = 1e-3
    render_step_size = 1.0 / 50  
    
    
    render_steps = int(math.ceil(render_step_size / step_size))
    
    
    time = 0
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    render_frame = 0
    
    print("Starting simulation...")
    print("Use WASD keys to control the vehicle:")
    print("W/S - Throttle/Brake")
    print("A/D - Steering")
    print("ESC - Exit simulation")
    
    
    while vis.Run():
        time = vehicle.GetSystem().GetChTime()
        
        
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_frame += 1
        
        
        driver_inputs = driver.GetInputs()
        
        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)
        
        
        manager.Update()
        
        
        if step_number % 100 == 0:  
            
            imu_data = imu.GetMostRecentBuffer()
            if imu_data.HasData():
                imu_buffer = sens.ChIMUBuffer(imu_data)
                acc_data = imu_buffer.GetAccelData()
                gyro_data = imu_buffer.GetGyroData()
                mag_data = imu_buffer.GetMagnetData()
                
                print(f"Time: {time:.2f}s")
                print(f"IMU - Accel: ({acc_data[0].x:.3f}, {acc_data[0].y:.3f}, {acc_data[0].z:.3f}) m/s²")
                print(f"IMU - Gyro: ({gyro_data[0].x:.3f}, {gyro_data[0].y:.3f}, {gyro_data[0].z:.3f}) rad/s")
            
            
            gps_data = gps.GetMostRecentBuffer()
            if gps_data.HasData():
                gps_buffer = sens.ChGPSBuffer(gps_data)
                coords = gps_buffer.GetGPSData()
                if len(coords) > 0:
                    print(f"GPS - Lat: {coords[0].Latitude:.6f}°, Lon: {coords[0].Longitude:.6f}°, Alt: {coords[0].Altitude:.3f}m")
        
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)
        
        
        step_number += 1
        
        
        realtime_timer.Spin(step_size)
    
    print("Simulation ended.")
    return 0

if __name__ == "__main__":
    main()