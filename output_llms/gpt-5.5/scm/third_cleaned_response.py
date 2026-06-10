import math
import random

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens





chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")
sens.SetSensorDataPath(chrono.GetChronoDataPath() + "sensor/")





init_x = -8.0
init_y = 0.0
init_z = 0.6

initLoc = chrono.ChVector3d(init_x, init_y, init_z)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0.0
terrainLength = 20.0
terrainWidth = 20.0
terrain_resolution = 0.02


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50.0





vehicle = veh.HMMWV_Full()
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





terrain = veh.SCMTerrain(vehicle.GetSystem())

terrain.SetSoilParameters(
    2e6,   
    0.0,   
    1.1,   
    0.0,   
    30.0,  
    0.01,  
    2e8,   
    3e4    
)


terrain.AddMovingPatch(
    vehicle.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1)
)

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(terrainLength, terrainWidth, terrain_resolution)





random.seed(10)

box_material = chrono.ChContactMaterialSMC()
box_material.SetFriction(0.8)
box_material.SetRestitution(0.1)
box_material.SetYoungModulus(2.0e7)

num_boxes = 18
boxes = []



vehicle_exclusion_half_length = 4.5
vehicle_exclusion_half_width = 2.5

for i in range(num_boxes):
    placed = False

    for attempt in range(200):
        sx = random.uniform(0.4, 1.2)
        sy = random.uniform(0.4, 1.2)
        sz = random.uniform(0.3, 1.0)

        x = random.uniform(-terrainLength / 2.0 + sx, terrainLength / 2.0 - sx)
        y = random.uniform(-terrainWidth / 2.0 + sy, terrainWidth / 2.0 - sy)

        
        inside_vehicle_start_zone = (
            abs(x - init_x) < vehicle_exclusion_half_length + sx / 2.0 and
            abs(y - init_y) < vehicle_exclusion_half_width + sy / 2.0
        )

        if inside_vehicle_start_zone:
            continue

        
        too_close_to_existing_box = False
        for existing_box in boxes:
            p = existing_box.GetPos()
            dx = x - p.x
            dy = y - p.y
            if math.sqrt(dx * dx + dy * dy) < 1.2:
                too_close_to_existing_box = True
                break

        if too_close_to_existing_box:
            continue

        box = chrono.ChBodyEasyBox(
            sx,
            sy,
            sz,
            800.0,          
            True,           
            True,           
            box_material
        )

        box.SetPos(chrono.ChVector3d(x, y, terrainHeight + sz / 2.0))
        box.SetRot(chrono.QuatFromAngleZ(random.uniform(-math.pi, math.pi)))

        
        box.SetFixed(True)

        vehicle.GetSystem().Add(box)
        boxes.append(box)

        placed = True
        break

    if not placed:
        print(f"Warning: could not place box {i} without violating the vehicle exclusion zone.")





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Demo with Random Boxes and Camera Sensor")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
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





manager = sens.ChSensorManager(vehicle.GetSystem())


manager.scene.AddPointLight(
    chrono.ChVector3f(0.0, 0.0, 8.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    80.0
)

manager.scene.AddPointLight(
    chrono.ChVector3f(6.0, -5.0, 5.0),
    chrono.ChColor(0.8, 0.9, 1.0),
    60.0
)

manager.scene.AddPointLight(
    chrono.ChVector3f(-6.0, 5.0, 5.0),
    chrono.ChColor(1.0, 0.85, 0.7),
    60.0
)


camera_update_rate = 30.0
camera_width = 1280
camera_height = 720
camera_fov = math.radians(70.0)



camera_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.8, 0.0, 1.5),
    chrono.QuatFromAngleY(math.radians(5.0))
)

camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),
    camera_update_rate,
    camera_offset_pose,
    camera_width,
    camera_height,
    camera_fov
)

camera.SetName("Chassis Camera")
camera.SetLag(0.0)
camera.SetCollectionWindow(1.0 / camera_update_rate)


camera.PushFilter(sens.ChFilterVisualize(camera_width, camera_height, "Vehicle Camera Feed"))

manager.AddSensor(camera)





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

    
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1

    realtime_timer.Spin(step_size)