import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
from pychrono.vehicle import HMMWV
from pychrono.vehicle import SCMDeformableTerrain
from pychrono.vehicle import ChDriver


def main():
    
    
    
    
    print("Initializing PyChrono environment...")
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    system.SetSolverType(chrono.ChSolver.Type_SOR)
    system.SetMaxItersSolverSpeed(500)
    system.SetMaxItersSolverStab(200)
    system.SetTolSpeed(1e-6)
    system.SetTolForce(1e-5)
    
    
    timestep = 0.002  
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
    
    
    
    
    
    print("Creating SCM deformable terrain...")
    
    
    terrain = SCMDeformableTerrain(system)
    
    
    terrain_size_x = 200.0  
    terrain_size_z = 200.0  
    terrain_num_x = 200     
    terrain_num_z = 200
    
    terrain.SetTexture(veh.GetDataFile("terrain/textures/grass.png"),
                      8.0, 8.0)
    terrain.SetMeshWireFrame(False)
    
    
    terrain.Initialize(terrain_size_x, terrain_size_z, terrain_num_x, terrain_num_z)
    
    
    
    
    
    print("Configuring SCM soil parameters...")
    
    
    
    
    
    
    
    
    terrain.SetSoilParameters(
        
        Prak_bekker=170000,        
        Prak_sink=0.015,            
        K_shrink=1.0,               
        K_x=700000,                 
        K_y=800000,                 
        n_sink=1.1,                 
        
        
        K_elastic=700000,           
        alpha_elastic=0.1,          
        n_elastic=1.5,             
        
        
        coh_shear=5000,            
        coh_press=20000,           
        alpha_shear=0.3,           
        
        
        phi_friction=0.6,          
        eta_friction=0.01,         
        phi_shear=0.45,            
        K_slope=100000,            
        
        
        ero_depth=0.02,            
        ero_cohesion=500,         
        
        
        dt_terrain=0.002,          
        viscosity=0.01             
    )
    
    
    terrain.SetPlotLevel(0, -0.1, 0.1, 1.0)  
    terrain.EnableVehicleSurface(true)
    
    
    
    
    
    print("Creating HMMWV vehicle...")
    
    
    vehicle_location = chrono.ChVectorD(0, 2.5, 0)
    vehicle_orientation = chrono.ChQuaternionD(1, 0, 0, 0)
    
    
    vehicle = HMMWV(system)
    vehicle.SetChassisFixed(False)
    vehicle.SetChassisCollision(False)
    
    
    tire_type = veh.TireModelType_RIGID
    vehicle.SetTireType(tire_type)
    vehicle.SetTireCollisionType(veh.CollisionType_ALL)
    
    
    vehicle.Initialize(chrono.ChCoordsysD(vehicle_location, vehicle_orientation))
    
    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    chassis_mesh = veh.GetDataFile("hmmwv/hmmwv_chassis_mesh.obj")
    wheel_mesh = veh.GetDataFile("hmmwv/hmmwv_wheel_mesh.obj")
    
    
    
    
    
    print("Configuring moving patch feature...")
    
    
    patch_size_x = 15.0  
    patch_size_z = 15.0  
    
    
    def update_patch():
        
        chassis_pos = vehicle.GetChassis().GetPos()
        return chrono.ChVectorD(chassis_pos.x, chassis_pos.y, chassis_pos.z)
    
    terrain.SetMovingPatch(vehicle.GetChassisBody(), patch_size_x, patch_size_z)
    
    
    
    
    
    print("Setting up interactive driver system...")
    
    
    driver = veh.ChDriver(vehicle)
    
    
    driver.SetThrottleDelta(0.02)    
    driver.SetSteeringDelta(0.03)    
    driver.SetBrakingDelta(0.06)      
    
    
    driver_app = irr.ChIrrApp(
        system,
        "HMMWV on SCM Terrain",
        irr.dimension2du(1280, 720),
        irr.EWV_ERINOSCROLLBARS,
        False,  
        True    
    )
    
    driver_app.AddTypicalCamera(irr.vector3df(8, 6, -8), irr.vector3df(0, 2, 0))
    driver_app.AddTypicalLights(irr.vector3df(50, 100, 50), irr.vector3df(50, 80, -50))
    driver_app.AddTypicalLights(irr.vector3df(-50, 100, -50), irr.vector3df(-50, 80, 50))
    
    
    driver_app.AddVehicle(vehicle, chrono.ChColor(0.8, 0.2, 0.2))
    
    
    driver_app.AddCustomTerrain(terrain)
    
    
    terrain.SetPlotColorMap(chrono.VEHICLE_TERRAIN_COLOR_FALSECOLOR)
    
    
    
    
    
    print("Configuring simulation parameters...")
    
    
    simulation_fps = 50  
    simulation_time = 0  
    max_simulation_time = 60  
    
    
    render_frame_step = int(1.0 / (timestep * simulation_fps))
    
    
    
    
    
    print("Starting simulation at {} FPS...".format(simulation_fps))
    print("Use arrow keys to control: Throttle (Up/Down), Steering (Left/Right), Brake (Space)")
    print("Press 'Q' to quit\n")
    
    driver_app.Start()
    
    frame_count = 0
    
    while driver_app.GetDevice().run():
        
        current_time = system.GetChTime()
        
        
        if current_time > max_simulation_time:
            print(f"\nSimulation completed: {max_simulation_time} seconds reached")
            break
        
        
        driver_app.BeginScene(True, True, irr.SColor(255, 100, 100, 100))
        driver_app.DrawAll()
        
        
        terrain.Render(driver_app.GetVideoDriver())
        
        
        driver.Synchronize(current_time)
        
        
        steering = driver.GetSteering()
        throttle = driver.GetThrottle()
        braking = driver.GetBraking()
        
        vehicle.ApplySteering(steering)
        vehicle.ApplyThrottle(throttle)
        vehicle.ApplyBraking(braking)
        
        
        vehicle.Synchronize(current_time, steering, throttle, braking)
        
        
        terrain.Synchronize(current_time)
        
        
        system.DoStepDynamics(timestep)
        
        
        simulation_time += timestep
        
        
        if frame_count % render_frame_step == 0:
            driver_app.EndScene()
        
        frame_count += 1
        
        
        if frame_count % 500 == 0:
            chassis_pos = vehicle.GetChassis().GetPos()
            chassis_vel = vehicle.GetChassis().GetVel()
            speed = math.sqrt(chassis_vel.x**2 + chassis_vel.z**2)
            print(f"Time: {current_time:.2f}s | "
                  f"Position: ({chassis_pos.x:.2f}, {chassis_pos.y:.2f}, {chassis_pos.z:.2f}) | "
                  f"Speed: {speed*3.6:.2f} km/h | "
                  f"Steering: {steering:.2f} | "
                  f"Throttle: {throttle:.2f}")
    
    
    print("\nSimulation ended. Cleaning up...")
    vehicle.Close()
    terrain.Close()
    
    print("Done!")


if __name__ == "__main__":
    main()