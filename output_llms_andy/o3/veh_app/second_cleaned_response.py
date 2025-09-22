import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as irr           
import pychrono.vehicle as veh
import pychrono.sensor as sens




chrono.SetChronoDataPath(chrono.GetChronoDataFilepath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

print("Using vehicle data folder:", chrono.GetChronoDataPath() + 'vehicle/')




initLoc = chrono.ChVector3d(0, -5, 0.4)          
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type     = veh.VisualizationType_MESH
suspension_vis_type  = veh.VisualizationType_PRIMITIVES
steering_vis_type    = veh.VisualizationType_PRIMITIVES
wheel_vis_type       = veh.VisualizationType_NONE
tire_vis_type        = veh.VisualizationType_MESH


step_size       = 1e-3
tire_step_size  = step_size
render_step     = 1.0/50.0          
end_time        = 30                




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


print("Vehicle mass      :", gator.GetVehicle().GetMass())
print("Driveline type    :", gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type        :", gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type         :", gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print()


gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

system = gator.GetSystem()




terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)



patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    50, 50
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()




blue = chrono.ChColor(0.1, 0.2, 0.9)


box        = chrono.ChBodyEasyBox(1, 1, 1,         
                                  1000,            
                                  True,            
                                  True)            
box.SetBodyFixed(True)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.GetVisualShape(0).SetColor(blue)
system.Add(box)


cyl         = chrono.ChBodyEasyCylinder(0.5, 1.0,  
                                        1000,
                                        True,
                                        True)
cyl.SetBodyFixed(True)
cyl.SetPos(chrono.ChVector3d(0, 0, 1.5))
cyl.GetVisualShape(0).SetColor(blue)
system.Add(cyl)




driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()




update_rate        = 10                      
image_width        = 1280
image_height       = 720
fov                = 1.408                  

manager = sens.ChSensorManager(system)


intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0
)


cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-8.0, 0.0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
)

cam = sens.ChCameraSensor(
    gator.GetChassisBody(),      
    update_rate,                 
    cam_offset,                  
    image_width,
    image_height,
    fov                          
)
cam.SetName("Third-person camera")
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)


lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 2),      
    chrono.QUNIT
)

lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),    
    update_rate,               
    lidar_offset,              
    800,                       
    300,                       
    2 * math.pi,               
    100.0                      
)


lidar.SetVerticalFOVUpper( math.pi / 12.0)   
lidar.SetVerticalFOVLower(-math.pi / 6.0)    


lidar.SetBeamShape(sens.LidarBeamShape.RECTANGULAR)
lidar.SetSampleRadius(2)
lidar.SetDivergenceAngle(0.003)
lidar.SetReturnMode(sens.LidarReturnMode.STRONGEST_RETURN)

lidar.SetName("Roof-mounted LiDAR")


lidar.PushFilter(sens.ChFilterLidarDepth())
lidar.PushFilter(sens.ChFilterLidarIntensity())
lidar.PushFilter(sens.ChFilterXYZIPointCloud())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, "LiDAR Point Cloud"))

manager.AddSensor(lidar)




realtime_timer = chrono.ChRealtimeStepTimer()
time = 0.0

while time < end_time:
    time = system.GetChTime()

    
    
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)

    inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, inputs, terrain)

    
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    realtime_timer.Spin(step_size)