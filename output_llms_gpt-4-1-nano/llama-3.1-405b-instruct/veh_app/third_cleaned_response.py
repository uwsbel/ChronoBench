import pychrono.core as chrono
import pychrono.irrlicht as irr
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


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 1000


render_step_size = 1.0 / 50  


noise_model = "NONE"  


update_rate = 10


image_width = 1280
image_height = 720


fov = 1.408


lag = 0


exposure_time = 0


vis = True


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
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")


gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()


box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.SetFixed(True)
box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().AddBody(box)


cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y ,0.5, 1, 1000)
cylinder.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder.SetFixed(True)
cylinder.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().AddBody(cylinder)


driver = veh.ChDriver(gator.GetVehicle())

driver.Initialize()




manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)


offset_pose = chrono.ChFramed(chrono.ChVector3d(-8.0, 0, 1.45), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov
)
cam.SetName("Third Person POV")

cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)


offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0.0, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),              
    update_rate,            
    offset_pose,            
    800,     
    300,       
    2 * chrono.CH_PI,         
    chrono.CH_PI / 12,         
    -chrono.CH_PI / 6,         
    100.0,                  
    sens.LidarBeamShape_RECTANGULAR,  
    2,          
    0.003,       
    0.003,       
    sens.LidarReturnMode_STRONGEST_RETURN             
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(1/update_rate)

lidar.PushFilter(sens.ChFilterDIAccess())

lidar.PushFilter(sens.ChFilterPCfromDepth())

lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

manager.AddSensor(lidar)


offset_pose = chrono.ChFramed(chrono.ChVector3d(-5.0, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
depth_cam = sens.ChDepthCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov,
    30  
)
depth_cam.SetName("Depth Camera")
depth_cam.PushFilter(sens.ChFilterVisualizeDepthMap(image_width, image_height, "Depth Map"))
manager.AddSensor(depth_cam)




realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)

    manager.Update()

    
    vehicle_state = gator.GetVehicle().GetChassisBody().GetFrame_COG_to_abs()
    print(f"Time: {time:.2f}, Position: ({vehicle_state.GetPos().x():.2f}, {vehicle_state.GetPos().y():.2f}, {vehicle_state.GetPos().z():.2f}), Heading: {vehicle_state.GetRot().GetEuler().z():.2f}")

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    
    realtime_timer.Spin(step_size)