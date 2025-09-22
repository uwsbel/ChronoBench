import pychrono.core as chrono
import pychrono.irrlicht as irr     
import pychrono.vehicle as veh
import pychrono.sensor as sens    
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
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


patch_mat = chrono.ChMaterialSurfaceNSC()  
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth
)


patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 20, 20)  
patch.SetColor(chrono.ChColor(0.3, 0.5, 0.3))  
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())



manager = sens.SensorManager(vehicle.GetSystem())



light_pos1 = chrono.ChVectorF(5, 5, 5)
vis.GetDevice().getSceneManager().addLightSceneNode(
    None,  
    irr.core.vector3df(light_pos1.x, light_pos1.y, light_pos1.z),
    irr.video.SColorf(1.0, 1.0, 1.0, 1.0),  
    15.0                                     
)
light_pos2 = chrono.ChVectorF(-5, -5, 5)
vis.GetDevice().getSceneManager().addLightSceneNode(
    None,
    irr.core.vector3df(light_pos2.x, light_pos2.y, light_pos2.z),
    irr.video.SColorf(0.8, 0.8, 0.8, 1.0),
    12.0
)



chassis_body = vehicle.GetVehicle().GetChassisBody()



camera_pos_rel = chrono.ChFrameD(chrono.ChVectorD(1.2, 0, 1.2))  


fov = sense_degrees_to_radians = math.radians(60)  

cam_width = 1280
cam_height = 720

camera = sens.CameraSensor(
    body=chassis_body,
    update_rate=1 / step_size,  
    offset_pose=camera_pos_rel,
    width=cam_width,
    height=cam_height,
    fov=fov
)


vis_filter = sens.ImageFilterRGBA()
camera.pushFilter(vis_filter)


manager.AddSensor(camera)



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

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)