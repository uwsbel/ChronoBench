import pychrono as chrono
import pychrono.vehicle as chronoveh
import pychrono.irrlicht as chrono_irr
import os
import math





if 'CHRONO_DATA_DIR' not in os.environ:
    
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    chrono_data_path = os.path.join(script_dir, "..", "..", "..", "chrono_data", "")
    if os.path.exists(os.path.join(chrono_data_path, "vehicle")):
        os.environ['CHRONO_DATA_DIR'] = chrono_data_path
    else:
        print("Error: CHRONO_DATA_DIR environment variable not set.")
        print("Please set CHRONO_DATA_DIR to the location of your Chrono data files.")
        exit(1)

chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
chronoveh.SetDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'vehicle', ''))





step_size = 0.005  


time_end = 100  


render_fps = 50.0
render_step_size = 1.0 / render_fps 


initLoc = chrono.ChVectorD(0, 0.7, 0) 
initRot = chrono.ChQuaternionD(1, 0, 0, 0) 


contact_method = chrono.ChContactMethod_SMC 


chassis_vis_type = chronoveh.VisualizationType_MESH
suspension_vis_type = chronoveh.VisualizationType_PRIMITIVES
steering_vis_type = chronoveh.VisualizationType_PRIMITIVES
wheel_vis_type = chronoveh.VisualizationType_MESH
tire_vis_type = chronoveh.VisualizationType_MESH


terrain_height = 0.0 
terrain_size_x = 200 
terrain_size_z = 200 




print("Creating Chrono system...")
if contact_method == chrono.ChContactMethod_SMC:
    sys = chrono.ChSystemSMC()
else:
    sys = chrono.ChSystemNSC()

sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) 
sys.SetSolverMaxIterations(150)
sys.SetMaxPenetrationRecoverySpeed(4.0)


print("Creating ARTcar vehicle...")
vehicle = chronoveh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chronoveh.CollisionType_NONE) 


vehicle.Initialize(chrono.ChCoordsysD(initLoc, initRot))


vehicle.SetChassisVisualizationType(chassis_vis_type)
vehicle.SetSuspensionVisualizationType(suspension_vis_type)
vehicle.SetSteeringVisualizationType(steering_vis_type)
vehicle.SetWheelVisualizationType(wheel_vis_type)






print(f"Vehicle initialized. Mass: {vehicle.GetVehicleMass()} kg")




print("Creating rigid terrain...")
terrain = chronoveh.RigidTerrain(sys)


if contact_method == chrono.ChContactMethod_SMC:
    patch_material = chrono.ChMaterialSurfaceSMC()
    patch_material.SetFriction(0.9)
    patch_material.SetRestitution(0.01)
    patch_material.SetYoungModulus(2e7)
    patch_material.SetPoissonRatio(0.3)
else: 
    patch_material = chrono.ChMaterialSurfaceNSC()
    patch_material.SetFriction(0.9)
    patch_material.SetRestitution(0.01)


patch = terrain.AddPatch(patch_material,
                         chrono.CSYSNORM, 
                         terrain_size_x, terrain_size_z,
                         terrain_height) 


texture_file = chronoveh.GetDataFile("terrain/textures/tile4.jpg")
patch.SetTexture(texture_file, 200, 200) 


patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

terrain.Initialize()
print("Terrain initialized.")




print("Creating Irrlicht application and driver...")
app = chrono_irr.ChIrrApp(sys, "ARTcar on Rigid Terrain", chrono_irr.dimension2du(1280, 720))
app.SetHUDDisplay(True) 


driver = chronoveh.ChIrrGuiDriver(app)


driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.04) 
driver.SetBrakingDelta(0.10) 


driver.Initialize()




trackPoint = chrono.ChVectorD(0.0, 0.0, 0.0) 

chase_dist = vehicle.GetOptimalChaseDistance() if hasattr(vehicle, 'GetOptimalChaseDistance') else 8.0
chase_height = vehicle.GetOptimalChaseHeight() if hasattr(vehicle, 'GetOptimalChaseHeight') else 2.0

driver.SetChaseCamera(trackPoint, chase_dist, chase_height)
driver.SetChaseCameraState(chronoveh.ChChaseCamera.Track) 
driver.SetChaseCameraAngle(-math.pi / 8.0) 




app.AssetBindAll()
app.AssetUpdateAll()




print(f"Starting simulation. Target FPS: {render_fps}. Physics step: {step_size}s.")
print("Controls: ")
print(" W/S: Throttle/Brake")
print(" A/D: Steering Left/Right")
print(" Mouse or Arrow Keys: Camera control (depending on camera mode)")
print(" C: Cycle camera modes")

realtime_timer = chrono.ChRealtimeStepTimer()
simulation_time = 0.0

while app.GetDevice().run():
    current_sim_time = sys.GetChTime()

    
    
    app.BeginScene(True, True, chrono.ChColor(0.1, 0.2, 0.3)) 
    
    
    driver_inputs_for_hud = driver.GetInputs()
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs_for_hud)
    app.DrawAll()
    app.EndScene()

    
    
    
    target_sim_time_for_next_render = simulation_time + render_step_size

    while sys.GetChTime() < target_sim_time_for_next_render:
        
        
        time_now = sys.GetChTime()
        actual_physics_step = min(step_size, target_sim_time_for_next_render - time_now)
        
        if actual_physics_step < 1e-7: 
            break

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time_now) 
        vehicle.Synchronize(time_now, driver_inputs, terrain) 
        terrain.Synchronize(time_now) 
        

        
        
        
        driver.Advance(actual_physics_step)
    
    simulation_time += render_step_size

    
    realtime_timer.Spin(render_step_size)

    if sys.GetChTime() > time_end:
        print("Simulation ended.")
        break


print("Exiting simulation.")