import pychrono.core as chrono

import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

print(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, -5, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE 
tire_vis_type = veh.VisualizationType_MESH


step_size = 1e-3
tire_step_size = step_size 



end_time = 30 


update_rate = 10        
image_width = 1280      
image_height = 720      
fov = 1.408             
lag = 0                 



gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY) 
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)


print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(0, veh.LEFT).GetTemplateName()) 
print("Tire type:      " + gator.GetVehicle().GetTire(0, veh.LEFT).GetTemplateName()) 
print("\n")


gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200) 
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) 
terrain.Initialize()


box = chrono.ChBodyEasyBox(1, 1, 1, 1000) 
box.SetPos(chrono.ChVector3d(5, 0, 0.5)) 
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.png"))
gator.GetSystem().AddBody(box)




cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y ,0.5, 1, 1000)
cylinder.SetPos(chrono.ChVector3d(7, 2, 1.0)) 
cylinder.SetFixed(True)
cylinder.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.png"))
gator.GetSystem().AddBody(cylinder)



driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()




manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0

manager.scene.GetBackground().color = chrono.ChColor4f(0.1, 0.2, 0.3, 1.0) 
manager.scene.AddPointLight(chrono.ChVector3f(20, 20, 100), chrono.ChColor3f(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(-20, 20, 100), chrono.ChColor3f(intensity, intensity, intensity), 500.0)



cam_offset_pose = chrono.ChFramed(chrono.ChVector3d(0.5, 0, 1.45), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))



rgb_cam = sens.ChCameraSensor(
    gator.GetChassisBody(), 
    update_rate,            
    cam_offset_pose,        
    image_width,            
    image_height,           
    fov                     
    
    
)
rgb_cam.SetName("RGB Camera")
rgb_cam.SetLag(lag)

rgb_cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "RGB Camera Feed"))
manager.AddSensor(rgb_cam)


lidar_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0.0, 0, 1.8), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)) 
    )
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),              
    update_rate,            
    lidar_offset_pose,      
    1800,                   
    32,                     
    2 * chrono.CH_PI,       
    chrono.CH_PI / 6,       
    -chrono.CH_PI / 6,      
    100.0,                  
    sens.LidarBeamShape_RECTANGULAR, 
    1,                      
    1,                      
    0.003,                  
    0.003,                  
    sens.LidarReturnMode_STRONGEST_RETURN, 
    0.01                    
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)

lidar.SetCollectionWindow(0) 
lidar.PushFilter(sens.ChFilterDIAccess()) 
lidar.PushFilter(sens.ChFilterPCfromDepth()) 
lidar.PushFilter(sens.ChFilterXYZIAccess()) 
if True: 
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 0.5, "Lidar Point Cloud")) 
manager.AddSensor(lidar)





depth_cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0, 2.0), 
    chrono.QUNIT 
                 
)
depth_cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    depth_cam_offset_pose,
    image_width,            
    image_height,           
    fov,                    
    
)
depth_cam.SetName("Depth Camera")
depth_cam.SetLag(lag)

depth_cam.SetClippingNearFar(0.1, 30.0) 


depth_cam.PushFilter(sens.ChFilterDepthAccess()) 
depth_cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Depth Map Visualization")) 
manager.AddSensor(depth_cam)





realtime_timer = chrono.ChRealtimeStepTimer()
sim_time = 0 







print("Starting simulation...")
while sim_time < end_time:
    current_time_gator = gator.GetSystem().GetChTime()

    
    
    steering_input = 0.5 * math.sin(2 * chrono.CH_PI * 0.1 * current_time_gator)
    throttle_input = 0.2
    braking_input = 0.0
    
    if current_time_gator > end_time / 2 : 
        throttle_input = 0.0
        braking_input = 0.5

    driver.SetSteering(steering_input)
    driver.SetThrottle(throttle_input)
    driver.SetBraking(braking_input)
    
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(current_time_gator)
    terrain.Synchronize(current_time_gator)
    gator.Synchronize(current_time_gator, driver_inputs, terrain)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size) 

    
    manager.Update()

    
    sim_time = gator.GetSystem().GetChTime()

    
    
    
    
    vehicle_pos = gator.GetVehicle().GetPos()
    vehicle_rot_quat = gator.GetVehicle().GetRot()
    
    
    
    
    vehicle_heading_rad = vehicle_rot_quat.GetYaw()
    vehicle_heading_deg = math.degrees(vehicle_heading_rad)

    
    log_string = (f"Time: {sim_time:.3f} s | "
                  f"Pos: ({vehicle_pos.x:.3f}, {vehicle_pos.y:.3f}, {vehicle_pos.z:.3f}) m | "
                  f"Heading: {vehicle_heading_deg:.2f} deg")
    print(log_string)

    
    

    
    realtime_timer.Spin(step_size)





print("Simulation finished.")