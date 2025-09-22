import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math






CHRONO_DATA_DIR = chrono.GetChronoDataPath()
if not CHRONO_DATA_DIR or CHRONO_DATA_DIR == "../data/": 
    
    
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    potential_path = os.path.join(script_dir, "..", "data", "") 
    if os.path.exists(os.path.join(potential_path, "vehicle", "hmmwv", "HMMWV_Vehicle.json")):
         CHRONO_DATA_DIR = potential_path
    else: 
        
        
        print("Warning: Chrono data directory might not be correctly set.")
        print(f"CHRONO_DATA_DIR detected as: {CHRONO_DATA_DIR}")
        print("Please ensure CHRONO_DATA_DIR environment variable is set or pychrono.SetChronoDataPath() is called with the correct path.")


chrono.SetChronoDataPath(CHRONO_DATA_DIR)
veh.SetDataPath(CHRONO_DATA_DIR + 'vehicle/')




path_radius = 40.0
path_center = chrono.ChVector3d(0, 0, 0.1)  



initLoc = chrono.ChVector3d(path_center.x + path_radius, path_center.y, 0.5)
initRot = chrono.ChQuaterniond()
initRot.SetFromAngleZ(math.pi / 2.0)  


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE  


tire_model = veh.TireModelType_TMEASY


terrain_height = 0.0  
terrainLength = 200.0  
terrainWidth = 100.0   


camera_trackPoint = chrono.ChVector3d(-4.0, 0.0, 1.7) 


contact_method = chrono.ChContactMethod_NSC


step_size = 2e-3  
tire_step_size = step_size


render_step_size = 1.0 / 50  


system = chrono.ChSystemNSC() 
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize(system) 

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system) 

patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




path_points_list = []
num_circle_points = 100  
for i in range(num_circle_points + 1):  
    angle = (2 * math.pi * i) / num_circle_points
    x = path_center.x + path_radius * math.cos(angle)
    y = path_center.y + path_radius * math.sin(angle)
    z = path_center.z
    path_points_list.append(chrono.ChVector3d(x, y, z))

path_curve = chrono.ChBezierCurve(path_points_list)


path_marker_radius = 0.6
path_marker_color = chrono.ChColor(0.8, 0.2, 0.2) 


marker1_pos = path_points_list[0]
path_display_marker1 = chrono.ChBodyEasySphere(path_marker_radius, 1000, True, False) 
path_display_marker1.SetPos(marker1_pos)
path_display_marker1.SetBodyFixed(True)
path_display_marker1.GetVisualShape(0).SetColor(path_marker_color)
system.Add(path_display_marker1)


marker2_pos = path_points_list[int(num_circle_points / 4)]
path_display_marker2 = chrono.ChBodyEasySphere(path_marker_radius, 1000, True, False)
path_display_marker2.SetPos(marker2_pos)
path_display_marker2.SetBodyFixed(True)
path_display_marker2.GetVisualShape(0).SetColor(path_marker_color)
system.Add(path_display_marker2)




target_speed_for_driver = 15.0  
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path_curve, "CircularPath", target_speed_for_driver, True) 


steering_controller = driver.GetSteeringController()
steering_controller.SetLookAheadDistance(6.0)  
steering_controller.SetGains(Kp=0.7, Ki=0.0, Kd=0.0)  


CONST_THROTTLE_VALUE = 0.3

driver.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(camera_trackPoint, 6.0, 0.5)  
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle()) 


sentinel_sphere_radius = 0.35
sentinel_vis_sphere = chrono.ChBodyEasySphere(sentinel_sphere_radius, 1000, True, False)
sentinel_vis_sphere.GetVisualShape(0).SetColor(chrono.ChColor(1, 0, 0))  
sentinel_vis_sphere.SetBodyFixed(True) 
system.Add(sentinel_vis_sphere)

target_sphere_radius = 0.35
target_vis_sphere = chrono.ChBodyEasySphere(target_sphere_radius, 1000, True, False)
target_vis_sphere.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 1))  
target_vis_sphere.SetBodyFixed(True) 
system.Add(target_vis_sphere)



print(f"VEHICLE MASS: {vehicle.GetVehicle().GetMass()}")

realtime_timer = chrono.ChRealtimeStepTimer()
simulation_steps = 0
render_frames_interval = math.ceil(render_step_size / step_size)

while vis.Run():
    current_time = system.GetChTime()

    
    if simulation_steps % render_frames_interval == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver.Synchronize(current_time)
    driver_inputs = driver.GetInputs()

    
    driver_inputs.m_throttle = CONST_THROTTLE_VALUE
    driver_inputs.m_braking = 0.0  

    
    terrain.Synchronize(current_time)
    vehicle.Synchronize(current_time, driver_inputs, terrain)
    vis.Synchronize(current_time, driver_inputs)

    
    
    active_steering_controller = driver.GetSteeringController()
    sentinel_pos_abs = active_steering_controller.GetSentinelPos()
    target_pos_abs = active_steering_controller.GetTargetPos()
    
    sentinel_vis_sphere.SetPos(sentinel_pos_abs)
    sentinel_vis_sphere.SetRot(chrono.QUNIT) 
    target_vis_sphere.SetPos(target_pos_abs)
    target_vis_sphere.SetRot(chrono.QUNIT)   

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size) 
    vis.Advance(step_size)     

    simulation_steps += 1
    realtime_timer.Spin(step_size)

del vis