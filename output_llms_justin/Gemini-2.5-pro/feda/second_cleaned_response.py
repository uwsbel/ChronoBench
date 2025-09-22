import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os



chrono_data_root = chrono.GetChronoDataPath()
if not chrono_data_root: 
    raise Exception("CHRONO_DATA_DIR environment variable not set or Chrono data path not found.")



chrono.SetChronoDataPath(chrono_data_root)
veh.SetDataPath(os.path.join(chrono_data_root, 'vehicle', '')) 


initLoc = chrono.ChVector3d(-50, 0, 0.5) 
initRot = chrono.ChQuaterniond(1, 0, 0, 0) 


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE 


tire_model = veh.TireModelType_TMEASY


terrainLength = 200.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC


step_size = 1e-3
tire_step_size = step_size 


render_step_size = 1.0 / 50  


vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False) 
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())




patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




path_start_pos = chrono.ChVector3d(initLoc.x, initLoc.y, 0.1) 
y_offset = 3.5                
lane_width_gate = 3.0         
straight_begin_len = 20.0     
straight_end_len = 20.0       
maneuver_segment_len = 60.0   
left_lane_change = False      

path = veh.DoubleLaneChangePath(
    path_start_pos,
    y_offset,
    lane_width_gate,        
    lane_width_gate,        
    lane_width_gate,        
    straight_begin_len,
    straight_end_len,
    maneuver_segment_len,
    left_lane_change
)


target_speed = 10.0  




steering_controller = veh.ChPathSteeringControllerStanley(path, vehicle.GetVehicle())
steering_controller.SetLookAheadDistance(5.0)
steering_controller.SetGain(0.8) 


driver = veh.ChPathFollowerDriver(
    vehicle.GetVehicle(),
    path,
    "DLC_Path",             
    target_speed,
    steering_controller     
)



driver.GetSpeedController().SetGains(0.5, 0.05, 0.0) 

driver.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle - Path Follower DLC')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5) 
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())



path_vis_body = chrono.ChBody()
path_vis_body.SetFixed(True) 
vehicle.GetSystem().Add(path_vis_body) 


path_asset = veh.ChPathShape(path)
path_asset.SetColor(chrono.ChColor(0.8, 0.0, 0.0)) 
path_asset.SetNumRenderPoints(400) 
path_vis_body.AddVisualShape(path_asset) 


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    vis.Synchronize(time, driver_inputs)


    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size) 

    
    step_number += 1

    
    realtime_timer.Spin(step_size)