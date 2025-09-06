import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens  
import numpy as np  
import math
import random


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
patch = terrain.AddPatch(
    patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth
)


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


manager = sens.ChSensorManager(vehicle.GetSystem())
manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(2, 2, 2), 5000)


lidar_offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 2), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),  
    10,                        
    lidar_offset_pose,         
    900,                       
    30,                        
    chrono.CH_PI / 2,          
    chrono.CH_PI / 6,          
    100.0,                     
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.1)
lidar.SetCollectionWindow(0.02)


lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 360, 1.0, "Lidar Point Cloud"))
manager.AddSensor(lidar)


def create_box(system, position, size, mass=1.0):
    body = chrono.ChBody()
    body.SetPos(position)
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVector3d(1,1,1))
    body_shape = chrono.ChVisualShapeBox(size.x, size.y, size.z)
    body.AddVisualShape(body_shape)
    body.GetCollisionModel().ClearModel()
    body.GetCollisionModel().AddBox(size.x/2, size.y/2, size.z/2)
    body.GetCollisionModel().BuildModel()
    body.EnableCollision(True)
    system.Add(body)
    return body


for _ in range(10):
    pos = chrono.ChVector3d(
        random.uniform(-20, 20),
        random.uniform(-20, 20),
        random.uniform(1, 3)
    )
    size = chrono.ChVector3d(
        random.uniform(0.5, 2.0),
        random.uniform(0.5, 2.0),
        random.uniform(0.5, 2.0)
    )
    create_box(vehicle.GetSystem(), pos, size, random.uniform(1, 10))


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
    
    
    if step_number % render_steps == 0:
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

import pychrono.sensor as sens
   import numpy as np
   import random

patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)

manager.Update()