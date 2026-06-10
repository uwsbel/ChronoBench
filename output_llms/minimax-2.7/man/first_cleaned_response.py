import os
import sys
import math
import numpy as np


import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr






SIMULATION_TIME = 30.0       
STEP_SIZE = 0.001            
OUTUPUT_FPS = 30             


VEHICLE_INITIAL_POS = chrono.ChVectorD(0, 1.0, 0)
VEHICLE_INITIAL_ROT = chrono.ChQuaternionD(1, 0, 0, 0)


TERRAIN_SIZE = 200.0         
TERRAIN_FRICTION = 0.8
TERRAIN_RESTITUTION = 0.05


TIRE_PRESSURE = 8.5e5        





def initialize_chrono_environment():
    
    
    chrono.SetDataPath(os.path.join(os.path.dirname(__file__), 
                                     'data', 'chrono', ''))
    veh.SetDataPath(os.path.join(os.path.dirname(__file__), 
                                  'data', 'vehicle', ''))
    
    
    print("=" * 60)
    print("PyChrono Vehicle Simulation")
    print("=" * 60)
    print(f"Chrono data path: {chrono.GetDataPath()}")
    print(f"Vehicle data path: {veh.GetDataPath()}")
    print("=" * 60)
    
    return chrono

def create_physical_system():
    
    
    system = chrono.ChSystemNSC()
    
    
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetMaxIterations(100)
    system.Set_TimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
    
    
    system.SetStepSize(STEP_SIZE)
    
    
    system.ChangeCollisionSystem(chrono.ChCollisionSystem.Type_BULLET)
    
    return system

def create_terrain(system):
    
    print("\n[INFO] Creating rigid terrain...")
    
    
    terrain = veh.RigidTerrain(system)
    
    
    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(TERRAIN_FRICTION)
    mat.SetRestitution(TERRAIN_RESTITUTION)
    mat.SetCompliance(0.0)
    mat.SetComplianceT(0.0)
    
    
    terrain_h = 0.5  
    
    patch = terrain.AddPatch(
        chrono.ChCoordsysD(chrono.ChVectorD(0, -terrain_h/2, 0), 
                          chrono.ChQuaternionD(1, 0, 0, 0)),
        mat,
        TERRAIN_SIZE, TERRAIN_SIZE,  
        terrain_h,
        0,  
        0   
    )
    
    
    try:
        tex_file = veh.GetChronoDataFile("terrain/concrete.jpg")
        if os.path.exists(tex_file):
            patch.SetTexture(tex_file, 5.0, 5.0)
            print(f"[INFO] Terrain texture applied: {tex_file}")
        else:
            
            patch.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
            print("[INFO] Using default terrain color (texture not found)")
    except:
        patch.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
        print("[INFO] Using default terrain color")
    
    
    patch.GetVisualizationShape().SetTexture(
        veh.GetChronoDataFile("terrain/texture.png"), 
        50, 50
    )
    
    
    terrain.Initialize()
    
    return terrain, patch

def create_truck(system):
    
    print("\n[INFO] Creating MAN 10t truck...")
    
    
    truck = veh.MAN_10t(system)
    
    
    truck.SetTireModelType(veh.TireModelType_TMEASY)
    
    
    tire_config = veh.TMeasyTireConfig()
    tire_config.Set TirePressure(TIRE_PRESSURE)
    truck.SetTireVisualization(veh.TireVisualizationType_MESH)
    
    
    truck.SetInitPosition(chrono.ChCoordsysD(
        VEHICLE_INITIAL_POS, 
        VEHICLE_INITIAL_ROT
    ))
    
    
    truck.SetDrivelineType(veh.DrivelineType_AWD)
    
    
    truck.SetTransmissionType(veh.TransmissionType_AUTOMATIC)
    
    
    truck.Initialize()
    
    print(f"[INFO] Truck initialized at position: {VEHICLE_INITIAL_POS}")
    print("[INFO] Using TMEASY tire model")
    print("[INFO] Driveline: AWD (All-Wheel Drive)")
    
    return truck

def create_driver(truck):
    
    
    driver = veh.ChDriver(keys_horizontal=irr.KEY_STEER, 
                         keys_vertical=[irr.KEY_ACCELERATE, irr.KEY_BRAKE])
    
    
    
    
    driver.Initialize()
    
    print("[INFO] Driver controller initialized")
    print("       Controls: Arrow Keys or WASD for steering/throttle/brake")
    
    return driver





def setup_visualization(system, truck):
    
    print("\n[INFO] Setting up Irrlicht visualization...")
    
    
    vis = irr.CChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowTitle("MAN 10t Truck - Rigid Terrain Simulation")
    vis.SetWindowSize(1280, 720)
    vis.SetStyle(irr.IrrVisualStyle_STEEL_BLUE)
    
    
    vis.Initialize()
    
    
    
    cam_offset = chrono.ChVectorD(-12.0, 5.0, 0.0)  
    vis.AddCameraRowWithMode(irr.CameraLocationType_FOLLOW, 
                             cam_offset,  
                             chrono.ChVectorD(0, 1.5, 0),  
                             1.5)  
    
    
    
    light_dir = chrono.ChVectorD(0.5, -1.0, -0.5)
    light_dir.Normalize()
    vis.AddLightDirectional(
        irr.vector3df(light_dir.x, light_dir.y, light_dir.z),
        irr.vector3df(1.0, 1.0, 1.0),
        0.8
    )
    
    
    vis.AddLightAmbient(irr.vector3df(0.3, 0.3, 0.3))
    
    
    
    vis.AddSkyBox()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    vis.SetShadows(True)
    
    print("[INFO] Visualization system initialized")
    print("       - Chase camera enabled")
    print("       - Directional lighting enabled")
    print("       - Skybox enabled")
    print("       - Shadows enabled")
    
    return vis

def add_custom_elements(vis):
    
    
    
    
    
    
    
    print("[INFO] Custom elements added to scene")





def run_simulation(system, truck, driver, vis):
    
    print("\n" + "=" * 60)
    print("STARTING SIMULATION")
    print("=" * 60)
    print("Press ESC or close window to exit")
    print("-" * 60)
    
    
    sim_time = 0.0
    step_number = 0
    render_frame = 0
    time_per_frame = 1.0 / OUTUPUT_FPS
    
    
    max_speed = 0.0
    distance_traveled = 0.0
    last_position = VEHICLE_INITIAL_POS
    
    
    hud_text = 
    
    
    while vis.Run():
        
        if sim_time >= render_frame * time_per_frame:
            vis.BeginScene()
            vis.Render()
            
            
            info_text = (
                f"Time: {sim_time:.2f}s | "
                f"Speed: {truck.GetSpeed()*3.6:.1f} km/h | "
                f"Throttle: {driver.GetThrottle()*100:.0f}% | "
                f"Brake: {driver.GetBraking()*100:.0f}%"
            )
            vis.AddCaption(info_text, irr.CaptionPosition_CENTER, 
                          irr.vector2df(0.5, 0.02), irr.SColorf(255, 255, 255, 1.0))
            
            vis.EndScene()
            render_frame += 1
        
        
        
        driver_inputs = driver.Synchronize(step_number)
        
        
        truck.Synchronize(step_number, sim_time, driver_inputs)
        
        
        system.Update()
        
        
        current_pos = truck.GetChassis().GetPos()
        current_speed = truck.GetSpeed()
        
        
        displacement = (current_pos - last_position).Length()
        distance_traveled += displacement
        last_position = current_pos
        
        
        if current_speed > max_speed:
            max_speed = current_speed
        
        
        sim_time = system.GetChTime()
        step_number += 1
        
        
        if step_number % 500 == 0:
            print(f"[{sim_time:6.2f}s] Speed: {current_speed*3.6:6.2f} km/h | "
                  f"Throttle: {driver_inputs.throttle*100:5.1f}% | "
                  f"Brake: {driver_inputs.brake*100:5.1f}%")
        
        
        
        if irr.key_is_pressed(irr.KEY_KEY_R):
            print("\n[INFO] Resetting simulation...")
            system.SetChTime(0)
            truck.Reset(chrono.ChCoordsysD(
                VEHICLE_INITIAL_POS, 
                VEHICLE_INITIAL_ROT
            ))
            sim_time = 0
            step_number = 0
            render_frame = 0
        
        
        if sim_time >= SIMULATION_TIME:
            print(f"\n[INFO] Simulation time reached ({SIMULATION_TIME}s)")
            break
        
        
        if current_pos.y < -10:
            print("\n[WARNING] Vehicle fell below terrain - possible instability")
            break
    
    
    print("\n" + "=" * 60)
    print("SIMULATION COMPLETE")
    print("=" * 60)
    print(f"Final Time:         {sim_time:.2f} s")
    print(f"Total Distance:    {distance_traveled:.2f} m")
    print(f"Max Speed:         {max_speed*3.6:.2f} km/h")
    print(f"Physics Steps:     {step_number}")
    print("=" * 60)
    
    return sim_time, distance_traveled, max_speed





def main():
    
    try:
        
        chrono = initialize_chrono_environment()
        
        
        system = create_physical_system()
        
        
        terrain, patch = create_terrain(system)
        
        
        truck = create_truck(system)
        
        
        driver = create_driver(truck)
        
        
        vis = setup_visualization(system, truck)
        
        
        add_custom_elements(vis)
        
        
        final_time, distance, max_speed = run_simulation(
            system, truck, driver, vis
        )
        
        print("\n[INFO] Simulation finished successfully!")
        return 0
        
    except KeyboardInterrupt:
        print("\n[INFO] Simulation interrupted by user")
        return 1
        
    except Exception as e:
        print(f"\n[ERROR] Simulation failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())