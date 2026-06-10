import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np
import os





def initialize_chrono_environment():
    
    
    chrono.SetDataPath(veh.GetDataPath())
    
    
    system = chrono.ChSystemNSC()
    
    
    system.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))  
    
    
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetMaxIterations(100)
    system.SetSolverMaxIterations(100)
    
    
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    return system


def create_vehicle_system(system, start_position):
    
    
    
    
    
    
    
    contact_method = chrono.ChContactMethod.NSC
    
    
    
    
    vehicle_orientation = chrono.ChQuaterniond(1, 0, 0, 0)
    
    
    collision_type = veh.CollisionType_BACKEND
    
    
    visualization_type = veh.VisualizationType_MESH
    
    
    drive_type = veh.DriveType.AWD
    
    
    powertrain_type = veh.PowertrainType_SHAFTS
    
    
    tire_model_type = veh.TireModelType_RIGID
    
    
    
    
    
    
    vehicle = veh.ARTcar(
        system,
        contact_method,
        vehicle_orientation,
        collision_type,
        visualization_type,
        drive_type,
        powertrain_type,
        tire_model_type
    )
    
    
    vehicle.Initialize(chrono.ChCoordsysd(start_position, vehicle_orientation))
    
    
    vehicle.SetChassisCollisionColor(chrono.ChColor(0.8, 0.2, 0.2))  
    vehicle.SetWheelVisibility(True)
    vehicle.SetSuspensionVisibility(True)
    
    return vehicle


def create_rigid_terrain(system, dimensions, texture_path=None):
    
    
    
    
    
    
    
    terrain_material = chrono.ChMaterialSurfaceNSC()
    terrain_material.SetFriction(0.8)           
    terrain_material.SetRestitution(0.1)         
    terrain_material.SetSpheroidal(0.0)          
    
    
    
    
    
    terrain_vis_mat = chrono.ChVisualMaterial()
    terrain_vis_mat.SetDiffuseColor(chrono.ChColor(0.5, 0.5, 0.5))  
    
    
    if texture_path and os.path.exists(texture_path):
        terrain_vis_mat.SetTexture(texture_path)
    
    
    
    
    
    
    terrain = veh.RigidTerrain(system)
    
    
    
    patch = terrain.AddPatch(
        terrain_material,
        dimensions[0],  
        dimensions[1],   
        dimensions[2],   
        chrono.ChVector3d(0, 0, 0)  
    )
    
    
    terrain.SetColor(chrono.ChColor(0.3, 0.6, 0.3))  
    terrain.SetTexture(veh.GetDataPath() + "terrain/textures/grass.jpg", 10, 10)
    
    
    patch.GetGroundBody().SetVisualMaterial(terrain_vis_mat)
    
    
    terrain.Initialize()
    
    return terrain


def create_interactive_driver(system, vehicle):
    
    
    
    
    
    
    
    driver = veh.ChDriver(vehicle)
    
    
    driver.Initialize()
    
    
    driver.SetSteeringDelta(0.02)      
    driver.SetThrottleDelta(0.05)      
    driver.SetBrakingDelta(0.05)       
    
    
    driver.SetMaxSteering(1.0)         
    driver.SetMaxThrottle(1.0)         
    driver.SetMaxBraking(1.0)         
    
    return driver


def create_visualization_system(vehicle, terrain):
    
    
    
    vis = irr.ChIrrApp(
        vehicle.GetSystem(),           
        "ARTcar Simulation",            
        irr.dimension2du(1280, 720),   
        irr.VerticalDir_Y              
    )
    
    
    vis.AddTypicalLights()                          
    vis.AddCamera(irr.vector3df(8, 6, -8))         
    vis.AddLogo(irr.recti(10, 10, 130, 40))       
    
    
    vis.AddVehicle(vehicle, irr.ChIrrAppTools_DRIVABLE)
    
    
    vis.AddTerrain(terrain)
    
    
    vis.SetShadows(True)
    
    
    vis.SetSkyBox(True)
    
    
    vis.AssetBindAll()
    vis.AssetUpdateAll()
    
    return vis


def run_simulation_loop(system, vehicle, driver, vis, terrain, fps=50):
    
    
    
    
    
    
    simulation_step = 1.0 / fps          
    visualization_step = 1.0 / fps      
    end_time = 100.0                    
    current_time = 0.0
    
    
    
    
    
    stats_interval = 1.0                
    last_stats_time = 0.0
    
    
    
    
    
    print("=" * 60)
    print("ARTcar Simulation Started")
    print("=" * 60)
    print(f"Target FPS: {fps}")
    print(f"Simulation step size: {simulation_step:.4f} seconds")
    print(f"End time: {end_time} seconds")
    print("-" * 60)
    print("Controls:")
    print("  [W/↑] - Accelerate")
    print("  [S/↓] - Brake/Reverse")
    print("  [A/←] - Steer Left")
    print("  [D/→] - Steer Right")
    print("  [SPACE] - Emergency Brake")
    print("  [ESC] - Exit Simulation")
    print("=" * 60)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    while current_time < end_time:
        
        
        
        
        
        if not vis.Run():
            print("\nSimulation terminated by user.")
            break
        
        
        if vis.GetDevice().getEventReceiver().IsKeyDown(irr.EKEY_ESCAPE):
            print("\nSimulation terminated (ESC pressed).")
            break
        
        
        
        
        
        
        steering_input = 0.0
        throttle_input = 0.0
        braking_input = 0.0
        
        
        if vis.GetDevice().getEventReceiver().IsKeyDown(irr.EKEY_KEY_W) or \
           vis.GetDevice().getEventReceiver().IsKeyDown(irr.EKEY_UP):
            throttle_input = 1.0
        if vis.GetDevice().getEventReceiver().IsKeyDown(irr.EKEY_KEY_S) or \
           vis.GetDevice().getEventReceiver().IsKeyDown(irr.EKEY_DOWN):
            braking_input = 1.0
        if vis.GetDevice().getEventReceiver().IsKeyDown(irr.EKEY_KEY_A) or \
           vis.GetDevice().getEventReceiver().IsKeyDown(irr.EKEY_LEFT):
            steering_input = 1.0
        if vis.GetDevice().getEventReceiver().IsKeyDown(irr.EKEY_KEY_D) or \
           vis.GetDevice().getEventReceiver().IsKeyDown(irr.EKEY_RIGHT):
            steering_input = -1.0
        
        
        driver.SetSteeringInput(steering_input)
        driver.SetThrottleInput(throttle_input)
        driver.SetBrakingInput(braking_input)
        
        
        
        
        
        
        vehicle.Synchronize(current_time, driver.GetInputs(), terrain)
        
        
        vehicle.Update(current_time)
        
        
        system.Update(current_time, simulation_step)
        
        
        
        
        
        vis.BeginScene()
        vis.DrawAll()                  
        
        
        draw_info_overlay(vis, current_time, vehicle, driver)
        
        vis.EndScene()
        
        
        
        
        
        if current_time - last_stats_time >= stats_interval:
            print_stats(current_time, vehicle)
            last_stats_time = current_time
        
        
        current_time += simulation_step
    
    print("=" * 60)
    print(f"Simulation completed. Total time: {current_time:.2f} seconds")
    print("=" * 60)


def draw_info_overlay(vis, time, vehicle, driver):
    
    
    
    vis.DrawAll()
    
    
    


def print_stats(time, vehicle):
    
    
    
    chassis_pos = vehicle.GetChassis().GetPos()
    chassis_vel = vehicle.GetChassis().GetVel()
    speed = np.sqrt(chassis_vel.x**2 + chassis_vel.y**2 + chassis_vel.z**2)
    
    
    num_wheels = vehicle.GetNumWheels()
    
    print(f"\n[T={time:.2f}s] Vehicle Statistics:")
    print(f"  Position: ({chassis_pos.x:.2f}, {chassis_pos.y:.2f}, {chassis_pos.z:.2f})")
    print(f"  Speed: {speed * 3.6:.2f} km/h")
    print(f"  Driver inputs: Steer={driver.GetSteeringInput():.2f}, "
          f"Throttle={driver.GetThrottleInput():.2f}, "
          f"Brake={driver.GetBrakingInput():.2f}")


def main():
    
    
    print("\n" + "=" * 70)
    print("  PYCHRONO ARTCAR VEHICLE SIMULATION")
    print("=" * 70 + "\n")
    
    
    
    
    
    
    start_position = chrono.ChVector3d(0, 1.0, 0)  
    
    
    terrain_dimensions = (100.0, 100.0, 0.5)  
    
    
    target_fps = 50
    
    
    
    
    
    print("[1/5] Initializing PyChrono environment...")
    system = initialize_chrono_environment()
    
    print("[2/5] Creating ARTcar vehicle system...")
    vehicle = create_vehicle_system(system, start_position)
    
    print("[3/5] Creating rigid terrain...")
    terrain = create_rigid_terrain(system, terrain_dimensions)
    
    print("[4/5] Creating interactive driver system...")
    driver = create_interactive_driver(system, vehicle)
    
    print("[5/5] Creating visualization system...")
    vis = create_visualization_system(vehicle, terrain)
    
    print("\nAll components initialized successfully!\n")
    
    
    
    
    
    run_simulation_loop(system, vehicle, driver, vis, terrain, fps=target_fps)
    
    
    
    
    
    print("\nCleaning up simulation...")
    del vis
    del vehicle
    del terrain
    del system
    
    print("Simulation cleanup complete.")






def create_simplified_vehicle(system, position):
    
    
    
    chassis = chrono.ChBody()
    chassis.SetPos(position)
    chassis.SetMass(500)  
    chassis.SetInertiaXX(chrono.ChVector3d(100, 100, 100))
    system.Add(chassis)
    
    
    chassis_vis = chrono.ChVisualShapeBox(2.0, 1.0, 4.0)  
    chassis.AddVisualShape(chassis_vis)
    
    
    chassis_shape = chrono.ChCollisionShapeBox(2.0, 1.0, 4.0)
    chassis.AddCollisionShape(chassis_shape)
    chassis.EnableCollision(True)
    
    return chassis






def create_advanced_terrain(system):
    
    
    
    terrain = veh.RigidTerrain(system)
    
    
    main_patch = terrain.AddPatch(
        chrono.ChMaterialSurfaceNSC(),
        200.0, 200.0, 1.0,
        chrono.ChVector3d(0, -0.5, 0)
    )
    
    
    ramp_material = chrono.ChMaterialSurfaceNSC()
    ramp_material.SetFriction(0.7)
    
    
    ramp_length = 20.0
    ramp_width = 5.0
    ramp_angle = 15.0  
    
    
    ramp = chrono.ChBody()
    ramp.SetPos(chrono.ChVector3d(30, 2, 0))
    ramp.SetMass(1000)
    
    
    ramp_angle_rad = np.radians(ramp_angle)
    ramp_rot = chrono.ChQuaterniond()
    ramp_rot.Q_from_AngY(ramp_angle_rad)
    ramp.SetRot(ramp_rot)
    
    
    ramp.AddCollisionShape(chrono.ChCollisionShapeBox(ramp_width, 0.5, ramp_length))
    ramp.AddVisualShape(chrono.ChVisualShapeBox(ramp_width, 0.5, ramp_length))
    ramp.EnableCollision(True)
    
    system.Add(ramp)
    
    terrain.Initialize()
    
    return terrain






if __name__ == "__main__":
    main()