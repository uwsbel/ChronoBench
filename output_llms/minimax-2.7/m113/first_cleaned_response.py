import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


def main():
    
    
    
    
    
    
    
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")
    
    
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    simulation_step_size = 1e-3        
    simulation_end_time = 20.0         
    real_time_scale = 1.0              
    
    
    
    
    
    
    vehicle = veh.M113_Vehicle(my_system, veh.DrivelineType_AXLE_4WD)
    
    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetDrivelineVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    
    
    vehicle_init_pos = chrono.ChCoordsysD(
        chrono.ChVectorD(0, 0.8, 0),  
        chrono.ChQuaternionD(1, 0, 0, 0)  
    )
    
    
    vehicle.Initialize(vehicle_init_pos)
    
    
    vehicle.SetVehicleVel(chrono.ChVectorD(0, 0, 0))
    
    print("M113 Vehicle initialized successfully")
    
    
    
    
    
    
    terrain_length = 200.0  
    terrain_width = 200.0   
    terrain_thickness = 0.5
    
    
    terrain = veh.RigidTerrain(my_system)
    
    
    terrain_material = chrono.ChMaterialSurfaceNSC()
    terrain_material.SetFriction(0.9)        
    terrain_material.SetRestitution(0.05)     
    terrain_material.SetCohesion(0.0)        
    terrain_material.SetCompliance(0.0)      
    
    
    ground_patch = terrain.AddPatch(
        terrain_material,
        chrono.ChVectorD(0, -terrain_thickness, 0),  
        chrono.ChVectorD(0, 0, 1),                    
        terrain_length,                               
        terrain_width,                                
        1,                                            
        1                                            
    )
    
    
    ground_color = chrono.ChColor(0.6, 0.5, 0.3)
    ground_patch.SetColor(ground_color)
    
    
    terrain.Initialize()
    
    print("Rigid terrain created successfully")
    print(f"  - Friction coefficient: {terrain_material.GetFriction()}")
    print(f"  - Restitution coefficient: {terrain_material.GetRestitution()}")
    
    
    
    
    
    
    driver = veh.ChDriver(vehicle.GetVehicle())
    
    
    driver.Initialize()
    
    
    driver.SetMax steering angle(0.5)    
    driver.SetMax throttle(1.0)          
    driver.SetMax braking(1.0)           
    
    print("Driver system initialized successfully")
    
    
    
    
    
    
    application = chronoirr.ChIrrApp(
        my_system,                              
        "M113 Vehicle Simulation",              
        chronoirr.dimension2du(1280, 720),      
        chronoirr.EDENTIALITY_ENABLED,          
        chronoirr.ESDR_OPENGL,                 
        chronoirr.EWS_NONE,                    
        0                                       
    )
    
    
    application.AddTypicalSky()
    application.AddTypicalLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
    application.AddTypicalLights(
        chronoirr.vector3df(50, 100, 50),      
        chronoirr.vector3df(50, 100, -50),     
        150,                                    
        chronoirr.vector3df(0.3, 0.3, 0.3)      
    )
    
    
    camera_distance = 12.0
    camera_height = 5.0
    camera_position = chronoirr.vector3df(camera_distance, camera_height, camera_distance)
    camera_target = chronoirr.vector3df(0, 1, 0)
    
    application.AddTypicalCamera(camera_position, camera_target)
    
    
    application.AddTypicalGUI()
    
    
    application.Bind(vehicle.GetVehicle())
    application.BindAll()
    
    
    application.SetTimestep(simulation_step_size)
    
    
    application.SetVideoframeSave(False)
    application.SetVideoframeWriter(None)
    
    
    application.SetTimestepType(chronoirr.E_TimestepType_TENTH)
    
    print("Irrlicht visualization setup complete")
    print(f"  - Camera position: ({camera_distance}, {camera_height}, {camera_distance})")
    print(f"  - Camera target: (0, 1, 0)")
    
    
    
    
    
    print("\n" + "="*60)
    print("Starting simulation loop...")
    print("="*60 + "\n")
    
    
    application.Start()
    
    
    start_wall_time = chrono.GetTime()
    current_time = 0.0
    step_count = 0
    display_interval = 1.0  
    
    
    while application.GetDevice().run():
        
        
        if current_time >= simulation_end_time:
            break
        
        
        
        
        
        
        application.BeginScene(True, True, chronoirr.SColor(255, 100, 100, 100))
        
        
        application.DrawAll()
        
        
        application.Render()
        
        
        application.EndScene()
        
        
        
        
        
        
        steering_input = driver.GetSteering()      
        throttle_input = driver.GetThrottle()      
        braking_input = driver.GetBraking()        
        
        
        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = steering_input
        driver_inputs.m_throttle = throttle_input
        driver_inputs.m_braking = braking_input
        driver_inputs.m_clutch = 0.0               
        driver_inputs.m_handbrake = False          
        
        
        
        
        
        
        vehicle.Synchronize(current_time, driver_inputs, terrain)
        
        
        terrain.Synchronize(current_time)
        
        
        driver.Synchronize(current_time)
        
        
        
        
        
        
        vehicle.Advance(simulation_step_size)
        
        
        terrain.Advance(simulation_step_size)
        
        
        driver.Advance(simulation_step_size)
        
        
        my_system.DoStepDynamics(simulation_step_size)
        
        
        
        
        
        
        application.Advance(simulation_step_size)
        
        
        
        
        
        
        current_time += simulation_step_size
        step_count += 1
        
        
        if current_time >= step_count * display_interval:
            
            chassis_pos = vehicle.GetVehicle().GetChassis().GetPos()
            chassis_speed = vehicle.GetVehicle().GetSpeed()
            
            print(f"Time: {current_time:6.2f}s | "
                  f"Position: ({chassis_pos.x:7.2f}, {chassis_pos.y:6.2f}, {chassis_pos.z:7.2f}) | "
                  f"Speed: {chassis_speed:6.2f} m/s | "
                  f"Steps: {step_count}")
    
    
    
    
    
    end_wall_time = chrono.GetTime()
    total_wall_time = end_wall_time - start_wall_time
    simulated_time = current_time
    
    print("\n" + "="*60)
    print("SIMULATION COMPLETE")
    print("="*60)
    print(f"\nSimulation Duration: {simulated_time:.2f} seconds")
    print(f"Total Steps: {step_count}")
    print(f"Wall Clock Time: {total_wall_time:.2f} seconds")
    print(f"Average Step Time: {total_wall_time/step_count*1000:.3f} ms")
    print(f"Real-time Factor: {simulated_time/total_wall_time:.2f}x")
    
    
    final_chassis_pos = vehicle.GetVehicle().GetChassis().GetPos()
    final_chassis_speed = vehicle.GetVehicle().GetSpeed()
    
    print("\nFinal Vehicle State:")
    print(f"  Position: ({final_chassis_pos.x:.2f}, {final_chassis_pos.y:.2f}, {final_chassis_pos.z:.2f})")
    print(f"  Speed: {final_chassis_speed:.2f} m/s")
    
    print("\n" + "="*60)
    print("End of Simulation")
    print("="*60)
    
    return 0


if __name__ == "__main__":
    main()