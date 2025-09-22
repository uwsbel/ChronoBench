import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
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


patch_mat = chrono.ChMaterialSurfaceNSC()  
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
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


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


sensor_manager = sens.ChSensorManager(vehicle.GetSystem())


lidar_scan_num_rays = 100  
lidar_num_scan_lines = 16  
lidar_horizontal_fov = 3.1415926  
lidar_vertical_fov = 0.261799  
lidar_max_distance = 50.0
lidar_frame_rate = 20  

lidar_offset_pose = chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 2.0),  
    chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))
)

lidar = sens.ChLidarSensor(
    parent=vehicle.GetChassisBody(),
    updateRate=lidar_frame_rate,
    offsetPose=lidar_offset_pose,
    numLidarPoints=lidar_scan_num_rays * lidar_num_scan_lines,
    rotationFrequency=lidar_frame_rate,
    horizontalFOV=lidar_horizontal_fov,
    verticalFOV=lidar_vertical_fov,
    maxDistance=lidar_max_distance,
    noiseStdDev=0.0,
    scanHeight=lidar_num_scan_lines,
    scanWidth=lidar_scan_num_rays
)

sensor_manager.AddSensor(lidar)


num_boxes = 15
box_size_range = (0.5, 2.0)  
box_height = terrainHeight + 0.5  
rng = np.random.default_rng()

for i in range(num_boxes):
    
    pos_x = rng.uniform(-terrainLength / 2 + 5, terrainLength / 2 - 5)
    pos_y = rng.uniform(-terrainWidth / 2 + 5, terrainWidth / 2 - 5)
    pos_z = box_height

    
    hx = rng.uniform(*box_size_range) / 2
    hy = rng.uniform(*box_size_range) / 2
    hz = rng.uniform(*box_size_range) / 2

    box = chrono.ChBodyEasyBox(2*hx, 2*hy, 2*hz,  
                               density=2000,       
                               visualize=True,
                               collide=True)
    box.SetPos(chrono.ChVectorD(pos_x, pos_y, pos_z + hz))  
    box.SetMass(100)  
    box.SetBodyFixed(False)
    box.GetCollisionModel().SetFamily(1)
    box.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1, 0)
    vehicle.GetSystem().Add(box)


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

    
    sensor_manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)