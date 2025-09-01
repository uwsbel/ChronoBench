import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono_data_path = chrono.GetChronoDataPath()
if not chrono_data_path:
    raise Exception("Chrono data path is not set. Please set CHRONO_DATA_PATH environment variable or set path manually.")
chrono.SetChronoDataPath(chrono_data_path)
veh.SetDataPath(chrono_data_path + 'vehicle/')



initLoc = chrono.ChVector3d(-40, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.UAZBUS()
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
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth)


patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0



t_start = 0.5     
t_first_lane_change_start = 1.0
t_first_lane_change_end = 3.0
t_return_lane_start = 3.0
t_return_lane_end = 5.0
t_second_lane_change_start = 5.0
t_second_lane_change_end = 7.0
t_braking_start = 7.0
t_braking_end = 10.0


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    steering_input = 0.0
    throttle_input = 0.8  
    braking_input = 0.0

    if t_first_lane_change_start <= time < t_first_lane_change_end:
        
        
        steering_input = 0.5 * (time - t_first_lane_change_start) / (t_first_lane_change_end - t_first_lane_change_start)
    elif t_return_lane_start <= time < t_return_lane_end:
        
        steering_input = 0.5 * (1 - (time - t_return_lane_start) / (t_return_lane_end - t_return_lane_start))
    elif t_second_lane_change_start <= time < t_second_lane_change_end:
        
        steering_input = 0.5 * (time - t_second_lane_change_start) / (t_second_lane_change_end - t_second_lane_change_start)
    elif t_braking_start <= time < t_braking_end:
        
        steering_input = 0.0
        throttle_input = max(0, 0.8 * (1 - (time - t_braking_start) / (t_braking_end - t_braking_start)))
        braking_input = (time - t_braking_start) / (t_braking_end - t_braking_start)
    elif time >= t_braking_end:
        
        steering_input = 0.0
        throttle_input = 0.0
        braking_input = 1.0

    
    steering_input = max(-1.0, min(1.0, steering_input))
    throttle_input = max(0.0, min(1.0, throttle_input))
    braking_input = max(0.0, min(1.0, braking_input))

    
    
    

    
    
    class CustomDriverInputs:
        def __init__(self, steering, throttle, braking):
            self.m_steering = steering
            self.m_throttle = throttle
            self.m_braking = braking

        def GetSteering(self):
            return self.m_steering

        def GetThrottle(self):
            return self.m_throttle

        def GetBraking(self):
            return self.m_braking

    custom_inputs = CustomDriverInputs(steering_input, throttle_input, braking_input)

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    
    vehicle.Synchronize(time, custom_inputs, terrain)
    vis.Synchronize(time, custom_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)