import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))






step_size = 0.005  


initLoc = chrono.ChVector3d(0, 0, 0.7)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0) 


contact_method = chrono.ChContactMethod_NSC


vehicle_file = veh.GetDataFile("MAN_10t/vehicle/MAN_10t_Vehicle.json")




powertrain_file = veh.GetDataFile("MAN_10t/powertrain/MAN_10t_SimpleMapPowertrain.json")


tire_model = veh.TireModelType_TMEASY


chassis_vis_type = veh.VisualizationType_MESH 
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH 
tire_vis_type = veh.VisualizationType_MESH 


chassis_collide = True



wheel_collide = False 


terrain_height = 0.0  
terrain_size_x = 200  
terrain_size_y = 200  
terrain_friction = 0.8
terrain_restitution = 0.01
terrain_texture_file = veh.GetDataFile("terrain/textures/Concrete.jpg")
terrain_texture_scale_x = 20 
terrain_texture_scale_y = 20 


irr_window_title = "MAN 10t TMEASY on Rigid Terrain"
irr_window_size = irr.dimension2du(1280, 720)
irr_logo_file = chrono.GetChronoDataFile('logo_pychrono_alpha.png')
irr_chase_cam_pos = chrono.ChVector3d(0.0, 0.0, 1.75) 
irr_chase_cam_dist = 8.0 
irr_chase_cam_height = 1.5 


driver_steering_delta = 0.02
driver_throttle_delta = 0.02
driver_braking_delta = 0.05




print("Creating Chrono system...")
sys = chrono.ChSystemNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.GetSettings().solver.max_iteration_bilateral = 150
sys.GetSettings().solver.max_iteration_normal = 0 
sys.GetSettings().solver.max_iteration_sliding = 150
sys.GetSettings().solver.max_iteration_spinning = 0 
sys.GetSettings().solver.alpha = 0 
sys.GetSettings().solver.contact_recovery_speed = 0.4
sys.SetMaxPenetrationRecoverySpeed(4.0)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.03)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.02)






print(f"Loading vehicle from: {vehicle_file}")
truck = veh.WheeledVehicle(sys, vehicle_file)
truck.Initialize(chrono.ChCoordsysd(initLoc, initRot))


print("Setting vehicle visualization...")
truck.SetChassisVisualizationType(chassis_vis_type)
truck.SetSuspensionVisualizationType(suspension_vis_type)
truck.SetSteeringVisualizationType(steering_vis_type)
truck.SetWheelVisualizationType(wheel_vis_type)
truck.SetTireVisualizationType(tire_vis_type) 


print("Setting vehicle collision...")
truck.SetChassisCollide(chassis_collide)
truck.SetWheelCollide(wheel_collide) 


print(f"Setting tire model to TMEASY...")
truck.SetTireType(tire_model)




print(f"Loading powertrain from: {powertrain_file}")
truck.CreatePowertrain(powertrain_file)




print("Initializing tires...")
truck.InitializeTires()



print("Creating rigid terrain...")
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChContactMaterialSMC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)



patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM, 
    terrain_size_x, terrain_size_y,
    terrain_height 
)

patch.SetTexture(terrain_texture_file, terrain_texture_scale_x, terrain_texture_scale_y)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5)) 

terrain.Initialize()




print("Creating Irrlicht visualization system...")
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowTitle(irr_window_title)
vis.SetWindowSize(irr_window_size)
vis.Initialize()


if os.path.exists(irr_logo_file):
    vis.AddLogo(irr_logo_file)
else:
    print(f"Warning: Logo file not found at {irr_logo_file}")


vis.AddSkyBox()




vis.AddTypicalLights(
    hemispheric = False, 
    
    
    
    
)





vis.SetChaseCamera(irr_chase_cam_pos, irr_chase_cam_dist, irr_chase_cam_height)





vis.BindAll()





print("Creating GUI driver...")
driver = veh.ChIrrGuiDriver(vis) 
driver.SetSteeringDelta(driver_steering_delta)
driver.SetThrottleDelta(driver_throttle_delta)
driver.SetBrakingDelta(driver_braking_delta)
driver.Initialize()
driver.AttachVehicle(truck.GetVehicle()) 




print("\nStarting simulation loop...")
print("------------------------------------")
print("Press T to increase throttle.")
print("Press G to decrease throttle.")
print("Press A to steer left.")
print("Press D to steer right.")
print("Press S to apply brakes.")
print("Press C to cycle camera modes (chase, fixed, inside).")
print("------------------------------------")


time = 0.0

while vis.Run():
    time = sys.GetChTime()

    
    vis.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.2)) 

    
    vis.Render()

    
    irr.ChIrrTools.drawGrid(vis, 1, 1, 50, 50,
                            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height + 0.01), chrono.QUNIT),
                            chrono.ChColor(0.1, 0.25, 0.25), True)

    
    driver.Synchronize(time)

    
    truck.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    terrain.Synchronize(time) 

    
    driver.Advance(step_size)
    truck.Advance(step_size)
    terrain.Advance(step_size) 
    sys.DoStepDynamics(step_size)

    
    vis.EndScene()

    if time > 300: 
        print("Simulation ended.")
        break

del sys 
print("Simulation finished.")