import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))






step_size = 0.005  
fps = 50           
render_step_size = 1.0 / fps 


init_loc = chrono.ChVectorD(0, 0.5, 0)  
init_rot = chrono.ChQuaternionD(1, 0, 0, 0) 


contact_method = chrono.ChContactMethod_SMC 


tire_model = veh.TireModelType_TMEASY


vis_type_chassis = veh.VisualizationType_MESH
vis_type_suspension = veh.VisualizationType_MESH
vis_type_steering = veh.VisualizationType_MESH
vis_type_wheel = veh.VisualizationType_MESH
vis_type_tire = veh.VisualizationType_MESH 



script_dir = os.path.dirname(__file__)
terrain_mesh_coll = os.path.join(script_dir, "data", "meshes", "Highway_col.obj")
terrain_mesh_vis = os.path.join(script_dir, "data", "meshes", "Highway_vis.obj")


if not os.path.exists(terrain_mesh_coll):
    print(f"Error: Collision mesh not found at {terrain_mesh_coll}")
    exit()
if not os.path.exists(terrain_mesh_vis):
    print(f"Error: Visualization mesh not found at {terrain_mesh_vis}")
    exit()




print("Creating Chrono system...")
sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetSolverMaxIterations(150)
sys.SetMaxPenetrationRecoverySpeed(4.0)




print("Creating HMMWV vehicle...")
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChFrameD(init_loc, init_rot))
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(step_size) 


hmmwv.SetChassisVisualizationType(vis_type_chassis)
hmmwv.SetSuspensionVisualizationType(vis_type_suspension)
hmmwv.SetSteeringVisualizationType(vis_type_steering)
hmmwv.SetWheelVisualizationType(vis_type_wheel)
hmmwv.SetTireVisualizationType(vis_type_tire)

hmmwv.Initialize()


powertrain = hmmwv.GetPowertrain()




print(f"Creating custom mesh terrain from: {terrain_mesh_coll} (collision) and {terrain_mesh_vis} (visual)")
terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT), 
                         terrain_mesh_coll, 
                         terrain_mesh_vis,  
                         0.8, 0.05)         

patch.SetContactMaterial(0.9, 0.01, 2e7, 0.3) 
terrain.Initialize()




print("Creating Irrlicht visualization...")
app = irr.ChIrrApp(hmmwv.GetSystem(), "HMMWV on Custom Mesh Terrain", irr.dimension2du(1280, 720))
app.SetSkyBox()
app.AddTypicalLights()
app.SetChaseCamera(hmmwv.GetChassisBody(), 6.0, 0.5) 
app.SetTimestep(step_size) 


app.AssetBindAll()
app.AssetUpdateAll()




print("Creating interactive driver...")
driver = veh.ChIrrGuiDriver(app)



steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()




print("\nStarting simulation... Press ESC to exit.")
print("Controls:")
print("  W/S: Throttle/Brake (forward/reverse)")
print("  A/D: Steer Left/Right")
print("  Space: Brake")
print("  Z: Toggle clutch (if manual transmission)")



render_steps = math.ceil(render_step_size / step_size)
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while app.GetDevice().run():
    time = hmmwv.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs) 

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    
    
    if step_number % render_steps == 0:
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192)) 
        app.DrawAll()
        app.EndScene()
        
        
        
        
        


    
    realtime_timer.Spin(step_size)
    step_number += 1

del app 
print("Simulation ended.")