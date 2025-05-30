import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math








chrono_data_dir = os.environ.get('CHRONO_DATA_DIR')
if chrono_data_dir is None:
    
    
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    chrono_data_dir = os.path.join(script_dir, "..", "..", "chrono_data", "")
    if not os.path.isdir(os.path.join(chrono_data_dir, "vehicle")):
        print("Error: CHRONO_DATA_DIR not set and default path not found.")
        print("Please set CHRONO_DATA_DIR or adjust the path in the script.")
        exit(1)

chrono.SetChronoDataPath(chrono_data_dir)
veh.SetDataPath(os.path.join(chrono_data_dir, 'vehicle', ''))


initLoc = chrono.ChVector3d(0, 0.7, 0)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0) 


step_size = 0.005  
render_fps = 50    
sim_time = 120     


contact_method = chrono.ChContactMethod_NSC 


tire_model = veh.TireModelType_TMEASY


vis_type_chassis = veh.VisualizationType_PRIMITIVES
vis_type_suspension = veh.VisualizationType_PRIMITIVES
vis_type_steering = veh.VisualizationType_PRIMITIVES
vis_type_wheel = veh.VisualizationType_PRIMITIVES
vis_type_tire = veh.VisualizationType_PRIMITIVES




print("Creating Chrono system...")
system = chrono.ChSystemNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)




print("Creating HMMWV vehicle...")
hmmwv = veh.hmmwv.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(veh.CollisionType_PRIMITIVES) 
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChFramed(initLoc, initRot))
hmmwv.SetPowertrainType(veh.PowertrainModelType_SIMPLE) 
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)             
hmmwv.SetTireType(tire_model)
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(vis_type_chassis)
hmmwv.SetSuspensionVisualizationType(vis_type_suspension)
hmmwv.SetSteeringVisualizationType(vis_type_steering)
hmmwv.SetWheelVisualizationType(vis_type_wheel)
hmmwv.SetTireVisualizationType(vis_type_tire)


vehicle = hmmwv.GetVehicle()

vehicle.SetSystem(system)





print("Creating rigid terrain...")
terrain = veh.RigidTerrain(system)


terrain_height = 0.0   
terrain_dim_x = 200.0  
terrain_dim_y = 200.0  


patch_mat = chrono.ChContactMaterialNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


patch = terrain.AddPatch(patch_mat,
                         chrono.ChVector3d(0, terrain_height, 0),  
                         chrono.ChVector3d(0, 1, 0),               
                         terrain_dim_x, terrain_dim_y)


texture_file = veh.GetDataFile("terrain/textures/tile4.jpg")
patch.SetTexture(texture_file, 200, 200) 
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5)) 

terrain.Initialize()




print("Creating Irrlicht visualization...")

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on Rigid Terrain Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5) 



vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


vis.BindAll()




print("Creating interactive driver...")
driver = veh.ChInteractiveDriverIRR(vis)



driver.SetSteeringDelta(0.02)  
driver.SetThrottleDelta(0.02)  
driver.SetBrakingDelta(0.05)   
driver.Initialize()




print("\nSimulation started. Control with:")
print("Steering: A/D keys")
print("Throttle: W key")
print("Braking: S key")
print("Camera: Use mouse and standard Irrlicht controls (e.g., WASD, QE for free camera)")
print("Press ESC to exit.\n")


realtime_timer = chrono.ChRealtimeStepTimer()
realtime_timer.SetStep(1.0 / render_fps) 


while vis.Run():
    time = system.GetChTime()

    
    vis.BeginScene()
    vis.Render()
    
    
    driver.DrawAllInfo() 
    
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain) 

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    system.DoStepDynamics(step_size)

    
    realtime_timer.Spin(step_size) 
    
    if time > sim_time : 
        print(f"Simulation time limit ({sim_time}s) reached.")
        break



print("Simulation ended.")


        
        render_fps = 50    
        step_size = 1.0 / render_fps 
        
        
        realtime_timer = chrono.ChRealtimeStepTimer() 
        
        
        
        
        realtime_timer.Spin(step_size) 


    
    target_fps = 50    
    step_size = 1.0 / target_fps 
    sim_time = 120     
    
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    
    
    
    realtime_timer.Spin(step_size)