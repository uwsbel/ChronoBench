import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import numpy as np


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



vehicle = veh.MAN_10t() 
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

patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
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


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


manager = sens.ChSensorManager(vehicle.GetSystem())


lidar_offset = chrono.ChVector3d(0, 0, 2.3)
lidar_theta = chrono.Q_FROM_AZ_EL(0, -0.5)
lidar_pose = chrono.ChFramed(lidar_offset, lidar_theta)
min_distance = 0.1
max_distance = 100
update_rate = 5
lag = 0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * np.pi
vertical_fov = np.pi / 6
max_grab_distance = 100
noise_model = sens.ChLidarNoiseModelNone()
return_mode = sens.ChLidarReturnMode_STRONGEST_RETURN

lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),           
    update_rate,            
    lag,                    
    lidar_pose,             
    horizontal_samples,     
    vertical_samples,       
    max_grab_distance,      
    horizontal_fov,         
    vertical_fov,           
    min_distance,           
    max_distance,           
    return_mode,            
    noise_model             
)
lidar.SetName("Lidar Sensor")
lidar.SetDescription("Lidar sensor mounted on the vehicle")
lidar.SetLag(lag)
lidar.SetUpdateRate(update_rate)
lidar.SetHorizontalSampleNum(horizontal_samples)
lidar.SetVerticalSampleNum(vertical_samples)
lidar.SetMaxHorizontalFOV(horizontal_fov)
lidar.SetMaxVerticalFOV(vertical_fov)
lidar.SetCollectionWindow(1.0/update_rate)
lidar.SetMinDistance(min_distance)
lidar.SetMaxDistance(max_distance)
lidar.SetBeamDivergence(0.0, 0.0)
lidar.SetPhaseFunction(sens.Ch_LAMBERT)
lidar.SetReturnMode(return_mode)
lidar.SetNoiseModel(noise_model)
lidar.SetVerticalRayArrangement(sens.ChVerticalSampleMode_CONSTANT, 0.0, 0)

assert manager.AddSensor(lidar)

        

body_list = []
num_boxes = 10  
box_half_dims = chrono.ChVector3d(3, 3, 3)  
box_mass = 100  
box_inertia = chrono.ChVector3d(10, 10, 10)  


for i in range(num_boxes):
    
    box_x = 3 * i

    
    box_body = chrono.ChBody()
    box_body.SetMass(box_mass)
    box_body.SetInertiaXX(box_inertia)
    box_body.SetFixed(False)  
    box_body.SetPos(chrono.ChVector3d(box_x, 0, 0))  

    
    box_shape = chrono.ChVisualShapeBox(2 * box_half_dims.x, 2 * box_half_dims.y, 2 * box_half_dims.z)
    box_shape.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
    box_body.AddVisualShape(box_shape)

    
    body_list.append(box_body)

    
    vehicle.GetSystem().Add(box_body)

manager.Initialize()
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()
    
    if (step_number % render_steps == 0) :
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