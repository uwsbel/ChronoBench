import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens




veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
print('Chrono data path  : ', chrono.GetChronoDataPath() + 'vehicle/')




init_loc = chrono.ChVectorD(0, -5, 0.4)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)




chassis_vis_type     = veh.VisualizationType_MESH
suspension_vis_type  = veh.VisualizationType_PRIMITIVES
steering_vis_type    = veh.VisualizationType_PRIMITIVES
wheel_vis_type       = veh.VisualizationType_NONE
tire_vis_type        = veh.VisualizationType_MESH




step_size      = 1e-3                 
tire_step_size = step_size
end_time       = 30.0                 




update_rate   = 10                    
image_width   = 1280
image_height  = 720
fov           = 1.408                 
lag           = 0
exposure_time = 0




gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
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


print('Vehicle mass     :', gator.GetVehicle().GetMass())
print('Driveline type   :', gator.GetVehicle().GetDriveline().GetTemplateName())
print('Brake type       :', gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print('Tire model       :', gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName(), '\n')

gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile('terrain/textures/tile4.jpg'), 50, 50)
terrain.Initialize()


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
box.SetPos(chrono.ChVectorD(0, 0, 0.5))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
gator.GetSystem().AddBody(box)

cyl = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.5, 1, 1000, True, True)
cyl.SetPos(chrono.ChVectorD(0, 0, 1.5))
cyl.SetFixed(True)
cyl.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
gator.GetSystem().AddBody(cyl)




driver = veh.ChDriver(gator.GetVehicle())   
driver.Initialize()




manager = sens.ChSensorManager(gator.GetSystem())

manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 100),
                            chrono.ChColor(1.0, 1.0, 1.0), 500)


rgb_pose = chrono.ChFrameD(chrono.ChVectorD(-8.0, 0, 1.45),
                           chrono.Q_from_AngAxis(0.2, chrono.ChVectorD(0, 1, 0)))
rgb_cam = sens.ChCameraSensor(gator.GetChassisBody(), update_rate, rgb_pose,
                              image_width, image_height, fov)
rgb_cam.SetName('RGB Camera')
rgb_cam.SetLag(lag)
rgb_cam.SetCollectionWindow(exposure_time)
rgb_cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, 'RGB Camera'))
manager.AddSensor(rgb_cam)


depth_pose = chrono.ChFrameD(chrono.ChVectorD(-5.0, 0, 2),
                             chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
depth_cam = sens.ChDepthCameraSensor(gator.GetChassisBody(), update_rate, depth_pose,
                                     image_width, image_height, fov)
depth_cam.SetName('Depth Camera')
depth_cam.SetLag(lag)
depth_cam.SetCollectionWindow(exposure_time)
depth_cam.SetMaxDepthRange(30.0)        
depth_cam.PushFilter(
    sens.ChFilterVisualizeDepth(image_width, image_height, 'Depth Map'))
manager.AddSensor(depth_cam)


lidar_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 2),
                             chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
lidar = sens.ChLidarSensor(gator.GetChassisBody(), update_rate, lidar_pose,
                           800, 300, 2*chrono.CH_PI,
                           chrono.CH_PI/12, -chrono.CH_PI/6, 100.0)
lidar.SetName('Lidar')
lidar.SetLag(lag)
lidar.SetCollectionWindow(1/update_rate)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(
    sens.ChFilterVisualizePointCloud(640, 480, 1.0, 'Lidar Point Cloud'))
manager.AddSensor(lidar)




realtime_timer = chrono.ChRealtimeStepTimer()
print('-----------------  Simulation started  -----------------')
current_time = 0.0
while current_time < end_time:
    current_time = gator.GetSystem().GetChTime()

    
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)

    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(current_time)
    terrain.Synchronize(current_time)
    gator.Synchronize(current_time, driver_inputs, terrain)

    
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    
    pos = gator.GetChassisBody().GetPos()
    yaw = chrono.Q_to_Euler123(gator.GetChassisBody().GetRot()).z  
    print(f't = {current_time:6.3f}  '
          f'pos = ({pos.x:7.3f}, {pos.y:7.3f}, {pos.z:5.2f})  '
          f'heading = {math.degrees(yaw):6.2f} deg')

    
    realtime_timer.Spin(step_size)

print('-----------------  Simulation finished  ----------------')