import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0.0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50.0  






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
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength,
    terrainWidth,
)


patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()






rng = np.random.default_rng(seed=42)

num_boxes = 30
box_density = 500.0

for i in range(num_boxes):
    box_x = float(rng.uniform(0.5, 2.0))
    box_y = float(rng.uniform(0.5, 2.0))
    box_z = float(rng.uniform(0.5, 2.5))

    
    pos_x = float(rng.uniform(8.0, terrainLength / 2.0 - 5.0))
    pos_y = float(rng.uniform(-terrainWidth / 2.0 + 5.0, terrainWidth / 2.0 - 5.0))
    pos_z = terrainHeight + box_z / 2.0

    box = chrono.ChBodyEasyBox(box_x, box_y, box_z, box_density, True, True)
    box.SetName(f"random_box_{i}")
    box.SetPos(chrono.ChVector3d(pos_x, pos_y, pos_z))

    
    box.SetFixed(True)

    vehicle.GetSystem().Add(box)






manager = sens.ChSensorManager(vehicle.GetSystem())


chassis_body = vehicle.GetVehicle().GetChassisBody()

lidar_update_rate = 10.0
lidar_horizontal_samples = 1024
lidar_vertical_samples = 16
lidar_horizontal_fov = 2.0 * math.pi
lidar_max_vert_angle = math.radians(10.0)
lidar_min_vert_angle = math.radians(-10.0)
lidar_max_distance = 80.0

lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0.0, 2.2),
    chrono.QUNIT,
)

lidar = sens.ChLidarSensor(
    chassis_body,
    lidar_update_rate,
    lidar_offset_pose,
    lidar_horizontal_samples,
    lidar_vertical_samples,
    lidar_horizontal_fov,
    lidar_max_vert_angle,
    lidar_min_vert_angle,
    lidar_max_distance,
)

lidar.SetName("Vehicle Lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)


lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())

manager.AddSensor(lidar)






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 10t Demo with Lidar and Random Boxes")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
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






print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())






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

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    manager.Update()

    
    step_number += 1

    
    realtime_timer.Spin(step_size)