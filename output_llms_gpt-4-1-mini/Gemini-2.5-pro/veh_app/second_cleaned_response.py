import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')






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
camera_fov = 1.408 


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

print("Brake type (axle 1, left): " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type (axle 1, left):  " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")


gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(), 200, 200) 
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) 
terrain.Initialize()




box_dims = chrono.ChVector3d(1, 1, 1)
box_pos = chrono.ChVector3d(0, 0, 0.5)
box_body = chrono.ChBodyEasyBox(box_dims.x, box_dims.y, box_dims.z, 
                               1000,       
                               False,      
                               True)       
box_body.SetPos(box_pos)
box_body.SetFixed(True) 
gator.GetSystem().Add(box_body)

box_visual_shape = chrono.ChVisualShapeBox(box_dims.x, box_dims.y, box_dims.z)

box_visual_shape.SetTexture(chrono.GetChronoDataPath() + "textures/bluewhite.png")
box_body.AddVisualShape(box_visual_shape)




cyl_radius = 0.5
cyl_height = 1.0
cyl_pos = chrono.ChVector3d(0, 0, 1.5) 

cyl_body = chrono.ChBodyEasyCylinder(cyl_radius, cyl_height,
                                    1000,       
                                    False,      
                                    True)       
cyl_body.SetPos(cyl_pos)

cyl_rot = chrono.ChQuaterniond()
cyl_rot.SetFromAngleAxis(chrono.CH_PI / 2, chrono.ChVector3d(1, 0, 0)) 
cyl_body.SetRot(cyl_rot)
cyl_body.SetFixed(True) 
gator.GetSystem().Add(cyl_body)

cyl_visual_shape = chrono.ChVisualShapeCylinder(cyl_radius, cyl_height) 
cyl_visual_shape.SetTexture(chrono.GetChronoDataPath() + "textures/bluewhite.png")
cyl_body.AddVisualShape(cyl_visual_shape, chrono.ChFramed()) 


driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()




manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0 

manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)


cam_offset_pose = chrono.ChFramed(chrono.ChVector3d(-8.0, 0, 1.45), chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
camera = sens.ChCameraSensor(
    gator.GetChassisBody(), 
    update_rate,            
    cam_offset_pose,        
    image_width,            
    image_height,           
    camera_fov              
)
camera.SetName("Third Person POV Camera")

camera.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(camera)




lidar_offset_pose = chrono.ChFramed(chrono.ChVector3d(0.0, 0, 2.0), chrono.QUNIT) 
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),    
    update_rate,               
    lidar_offset_pose,         
    800,                       
    300,                       
    2 * chrono.CH_PI,          
    -chrono.CH_PI / 6,         
    chrono.CH_PI / 12,         
    100.0,                     
    sens.LidarReturnMode_STRONGEST 
)
lidar.SetName("Lidar Sensor")
lidar.SetBeamShapeType(sens.ShapeType_RECTANGULAR) 
lidar.SetBeamDivergence(0.003) 


lidar.PushFilter(sens.ChFilterDIAccess()) 
lidar.PushFilter(sens.ChFilterPCXYZIAccess()) 



vis_filter_lidar = sens.ChFilterVisualizePointCloud(image_width, image_height, 2 * chrono.CH_PI, 100.0, "Lidar Point Cloud")
vis_filter_lidar.SetPointSize(2) 
lidar.PushFilter(vis_filter_lidar)

manager.AddSensor(lidar)




realtime_timer = chrono.ChRealtimeStepTimer()
time = 0

print("Starting simulation...")
while time < end_time:
    time = gator.GetSystem().GetChTime()

    
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)

    
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    

    
    realtime_timer.Spin(step_size)

print("Simulation finished.")