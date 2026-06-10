import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math

def main():
    
    
    
    
    
    
    print("=" * 60)
    print("PyChrono UAZBUS Vehicle Simulation")
    print("=" * 60)
    
    
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    
    system = chrono.ChSystemNSC()
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetMaxIterations(100)
    system.SetSystemTolerance(1e-6)
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    system.SetContactForceModel(chrono.ChContactForceModel.Hertz)
    
    
    
    
    
    
    step_size = 0.001  
    
    
    t_end = 100.0
    
    
    out_dir = "./uazbus_simulation_output/"
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)
    
    
    
    
    
    
    
    
    print("\n[INFO] Creating rigid terrain...")
    
    
    terrain_mat = chrono.ChMaterialSurfaceNSC()
    terrain_mat.SetFriction(0.8)           
    terrain_mat.SetRestitution(0.1)        
    terrain_mat.SetCohesion(0.0)          
    terrain_mat.SetSpinningFriction(0.0)   
    
    
    terrain = veh.RigidTerrain(system)
    
    
    terrain_length = 200.0   
    terrain_width = 100.0    
    terrain_thickness = 0.5  
    
    patch = terrain.AddPatch(
        terrain_mat,
        chrono.ChVectorD(0, -terrain_thickness/2, 0),  
        chrono.ChVectorD(0, 0, 0),                     
        terrain_length,
        terrain_width
    )
    
    
    patch.SetTexture(veh.GetDataFile("terrain/textures/grass.png"), 100, 100)
    
    
    terrain.Initialize()
    
    print(f"  - Terrain created: {terrain_length}m x {terrain_width}m")
    print(f"  - Friction: {terrain_mat.GetFriction()}")
    print(f"  - Restitution: {terrain_mat.GetRestitution()}")
    
    
    
    
    print("\n[INFO] Creating UAZBUS vehicle...")
    
    
    chassis_init_pos = chrono.ChVectorD(0, 1.0, 0)
    chassis_init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    initial_speed = 0.0  
    
    
    
    vehicle = veh.UAZBus(system)
    
    
    vehicle.SetChassisPos(chassis_init_pos)
    vehicle.SetChassisRot(chassis_init_rot)
    
    
    vehicle.Initialize()
    
    
    vehicle.SetInitWheelLinSpeed(initial_speed)
    
    print(f"  - Vehicle initialized at position: {chassis_init_pos}")
    print(f"  - Initial speed: {initial_speed} m/s")
    
    
    
    
    print("\n[INFO] Setting up powertrain...")
    
    
    powertrain = veh.ShafletsMAPTorqueModel(system, vehicle.GetDriveshaft())
    powertrain.Initialize(chrono.ChShaft(vehicle.GetDriveshaft()), 
                          chrono.ChShaft(vehicle.GetCrankshaft()), 
                          0.0,   
                          600.0) 
    vehicle.InitializePowertrain(powertrain)
    
    print("  - Powertrain initialized")
    
    
    
    
    print("\n[INFO] Initializing driver system...")
    
    
    driver = veh.Driver(vehicle)
    
    
    driver.SetSteering(0.0)           
    driver.SetThrottle(0.3)          
    driver.SetBraking(0.0)           
    
    
    
    
    
    driver.SetTimeDelay(0.1)  
    
    print("  - Driver system initialized")
    
    
    
    
    
    print("\n[INFO] Setting up Irrlicht visualization...")
    
    
    application = irr.ChIrrApp(
        system,
        "PyChrono UAZBUS Simulation",
        irr.dimension2du(1280, 720),
        irr.EWI_ABSOLUTE_SET | irr.EWI_PAUSE
    )
    
    
    application.AddTypicalLights(
        irr.E_LIGHT_TYPE.ELT_SUN,           
        irr.SColorf(1.0, 1.0, 0.9, 1.0),     
        irr.vector3df(100, 100, 50),         
        irr.vector3df(0, 0, 0)               
    )
    
    
    application.AddTypicalLights(
        irr.E_LIGHT_TYPE.ELT_FAST,           
        irr.SColorf(0.8, 0.8, 1.0, 1.0),     
        irr.vector3df(-50, 50, -50),          
        irr.vector3df(0, 0, 0)                
    )
    
    
    application.AddTypicalCamera(
        irr.vector3df(6.0, 3.0, -6.0),       
        irr.vector3df(0.0, 1.0, 0.0)         
    )
    
    
    application.AddTypicalSky()
    
    
    application.AddVehicle(vehicle, irr.vector3df(-1.0, 0.5, 0.0))
    
    
    application.AddTerrain(terrain)
    
    
    application.SetShadows(True)
    
    
    application.SetShowStats(True)           
    application.SetShowProfiler(True)        
    
    print("  - Irrlicht visualization initialized")
    print("  - Window size: 1280x720")
    
    
    
    
    
    print("\n" + "=" * 60)
    print("Starting Simulation...")
    print("=" * 60)
    print("\nControls:")
    print("  SPACE   - Pause/Resume simulation")
    print("  W       - Increase throttle")
    print("  S       - Decrease throttle/Brake")
    print("  A/D     - Steering left/right")
    print("  R       - Reset simulation")
    print("  ESC     - Exit simulation")
    print("\n" + "-" * 60)
    
    
    simulation_time = 0.0
    frame_count = 0
    last_fps_time = simulation_time
    
    
    application.BindSyncMesh()
    application.SetTimestep(step_size)
    
    
    while application.GetDevice().run():
        
        if t_end > 0 and simulation_time >= t_end:
            print(f"\n[INFO] Simulation completed at t = {simulation_time:.2f}s")
            break
        
        
        application.BeginScene(True, True, irr.SColor(255, 100, 100, 100))
        
        
        application.DrawAll()
        
        
        driver.Update(simulation_time)
        
        
        vehicle.Synchronize(simulation_time, driver.GetThrottle(), 
                           driver.GetSteering(), driver.GetBraking())
        
        
        terrain.Synchronize(simulation_time)
        
        
        application.DoStep()
        
        
        simulation_time += step_size
        frame_count += 1
        
        
        if frame_count % 1000 == 0:
            
            vehicle_state = vehicle.GetVehicle().GetState()
            speed_kmh = vehicle_state.speed * 3.6  
            
            print(f"  t = {simulation_time:7.2f}s | "
                  f"Speed: {speed_kmh:6.1f} km/h | "
                  f"Pos: ({vehicle_state.pos.x:7.2f}, "
                  f"{vehicle_state.pos.z:7.2f})")
        
        
        application.EndScene()
        
        
        if frame_count % 10 == 0:
            speed_kmh = vehicle.GetVehicle().GetState().speed * 3.6
            title = f"PyChrono UAZBUS | t={simulation_time:.1f}s | Speed: {speed_kmh:.1f} km/h"
            application.GetDevice().setWindowCaption(title)
    
    
    
    
    
    print("\n" + "=" * 60)
    print("Simulation Statistics")
    print("=" * 60)
    print(f"  Total simulation time: {simulation_time:.2f} seconds")
    print(f"  Total frames rendered: {frame_count}")
    print(f"  Average FPS: {frame_count / simulation_time:.1f}")
    print(f"  Final vehicle position: {vehicle.GetChassis().GetPos()}")
    
    
    final_state = vehicle.GetVehicle().GetState()
    print(f"  Final speed: {final_state.speed * 3.6:.1f} km/h")
    print(f"  Final orientation: {vehicle.GetChassis().GetRot()}")
    
    
    if vehicle.LogSubsystems():
        print(f"  Subsystem data logged to: {out_dir}")
    
    print("\n[INFO] Simulation terminated successfully.")
    
    return 0

if __name__ == "__main__":
    exit(main())