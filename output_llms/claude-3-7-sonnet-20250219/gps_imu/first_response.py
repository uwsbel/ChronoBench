import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import numpy as np
import math

def main():
    # Set the path to the Chrono data directory
    chrono.SetChronoDataPath('/path/to/chrono/data/')
    
    # Create a Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    # Initialize HMMWV vehicle
    # Create the terrain
    terrain = veh.RigidTerrain(system)
    patch_material = chrono.ChMaterialSurfaceNSC()
    patch_material.SetFriction(0.9)
    patch_material.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_material, 
                            chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                            200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()
    
    # Create the HMMWV vehicle, set parameters, and initialize
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()
    
    # Get chassis body from the vehicle
    chassis_body = vehicle.GetChassisBody()
    
    # Add visualization for the vehicle
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    # Create the driver system
    driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
    driver.SetHUDLocation(0, 0)
    driver.SetChaseCamera(6.0, 0.5, 0.5)
    driver.SetThrottleDelta(1.0 / 50)
    driver.SetSteeringDelta(1.0 / 50)
    driver.SetBrakingDelta(1.0 / 50)
    
    # Set the time response for steering keyboard inputs
    steering_time = 1.0
    driver.SetSteeringDelta(steering_time / 50)
    
    # Set driver mode
    driver.SetInputMode(veh.ChIrrGuiDriver.InputMode_KEYBOARD)
    
    # Initialize the driver system
    driver.Initialize()
    
    # Create the Irrlicht visualization system
    vis = chronoirr.ChIrrApp(system, "HMMWV Vehicle Simulation", chronoirr.dimension2du(1280, 720))
    vis.AddTypicalLogo()
    vis.AddTypicalSky()
    vis.AddTypicalLights()
    vis.AddTypicalCamera(chronoirr.vector3df(0, 0, 15))
    vis.SetSymbolscale(0.5)
    vis.SetShowInfos(True)
    vis.Initialize()
    
    # ----------------------------
    # Create and set up the sensors
    # ----------------------------
    
    # Create a manager for the sensors
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 500)
    
    # Create GPS sensor
    gps_offset = chrono.ChVectorD(0, 0, 0)
    gps_update_rate = 10.0  # Hz
    
    gps = sens.ChGPSSensor(
        chassis_body,              # body to which sensor is attached
        gps_update_rate,           # update rate
        gps_offset,                # offset from the body reference frame
        gps_noise=None,            # noise model
        gps_reference=None         # reference GPS position
    )
    
    # Filter sensor data
    gps.PushFilter(sens.ChFilterGPSAccess())
    
    # Add GPS sensor to the manager
    manager.AddSensor(gps)
    
    # Create IMU sensor
    imu_offset = chrono.ChVectorD(0, 0, 0)
    imu_update_rate = 100.0  # Hz
    
    imu = sens.ChIMUSensor(
        chassis_body,              # body to which sensor is attached
        imu_update_rate,           # update rate
        imu_offset,                # offset from the body reference frame
        imu_noise=None             # noise model
    )
    
    # Filter sensor data
    imu.PushFilter(sens.ChFilterIMUAccess())
    
    # Add IMU sensor to the manager
    manager.AddSensor(imu)
    
    # Initialize the sensor manager
    manager.Initialize()
    
    # Get vehicle mass
    vehicle_mass = vehicle.GetVehicle().GetVehicleMass()
    print(f"HMMWV vehicle mass = {vehicle_mass} kg")
    
    # -----------------
    # Simulation loop
    # -----------------
    
    step_size = 1e-3  # simulation step size
    render_step_size = 1.0 / 50  # time between visualization frames
    
    time = 0.0
    render_time = 0.0
    
    while vis.GetDevice().run():
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        
        # Update modules (vehicle, driver, terrain, sensors, visualization)
        time = vehicle.GetSystem().GetChTime()
        
        # Process driver inputs
        driver.Synchronize(time)
        
        # Update vehicle
        vehicle.Synchronize(time, driver_inputs, terrain)
        
        # Update terrain
        terrain.Synchronize(time)
        
        # Update sensor manager
        manager.Update()
        
        # Advance simulation for all modules
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        # Render scene and output vehicle information
        render_time += step_size
        
        if render_time >= render_step_size:
            vis.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
            vis.DrawAll()
            vis.EndScene()
            
            # Print sensor data
            if time > 1.0:  # Let the simulation stabilize first
                # Access GPS data
                gps_data = gps.GetMostRecentGPSBuffer()
                if gps_data.HasData():
                    latitude = gps_data.Latitude
                    longitude = gps_data.Longitude
                    altitude = gps_data.Altitude
                    print(f"GPS: Lat={latitude:.6f}, Lon={longitude:.6f}, Alt={altitude:.2f}")
                
                # Access IMU data
                imu_data = imu.GetMostRecentIMUBuffer()
                if imu_data.HasData():
                    acc = imu_data.Acc
                    gyro = imu_data.Gyro
                    print(f"IMU: Acc=({acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f}), Gyro=({gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f})")
            
            render_time = 0
        
        # Check if simulation should end
        if time > 60:  # End after 60 seconds
            break
    
    return 0

if __name__ == "__main__":
    main()