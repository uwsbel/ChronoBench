import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sensor
import math
import numpy as np

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation steps
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  # FPS = 50

# Create and initialize vehicle
vehicle = veh.MAN_5t()  # Corrected from MAN_10t to valid MAN_5t
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

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # Changed texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create random boxes
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.5)
for _ in range(20):
    box_body = chrono.ChBody()
    box_body.SetPos(chrono.ChVector3d(
        np.random.uniform(-terrainLength/2, terrainLength/2),
        np.random.uniform(-terrainWidth/2, terrainWidth/2),
        terrainHeight + 0.5
    ))
    box_body.SetFixed(True)
    
    # Visual shape
    box_shape = chrono.ChBoxShape(chrono.ChVector3d(1, 1, 1))
    box_shape.SetColor(chrono.ChColor(np.random.rand(), np.random.rand(), np.random.rand()))
    box_body.AddVisualShape(box_shape)
    
    # Collision shape
    collision_shape = chrono.ChCollisionShapeBox(box_mat, 1, 1, 1)
    box_body.AddCollisionShape(collision_shape)
    box_body.EnableCollision(True)
    vehicle.GetSystem().Add(box_body)

# Create sensor manager
sensor_manager = sensor.ChSensorManager(vehicle.GetSystem())
sensor_manager.scene.AddPointLight(chrono.ChVector3d(0, 0, 100), chrono.ChColor(1, 1, 1), 5000)

# Create lidar sensor
lidar_body = vehicle.GetVehicle().GetChassisBody()
lidar = sensor.ChLidarSensor(
    lidar_body,
    30,  # update rate
    chrono.ChFrameD(chrono.ChVector3d(0.5, 0, 1.5), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))),
    900,  # horizontal samples
    30,   # vertical channels
    chrono.CH_C_PI / 1.5,  # horizontal FOV
    chrono.CH_C_PI / 6,    # vertical FOV
    0.1,
    100.0
)
lidar.PushFilter(sensor.ChFilterPCfromDepth())
lidar.PushFilter(sensor.ChFilterVisualize(640, 480, "Lidar Data"))
lidar.PushFilter(sensor.ChFilterSavePtCloud("lidar_data/"))
sensor_manager.AddSensor(lidar)

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN Truck Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    # Update modules
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    sensor_manager.Update()  # Added sensor manager update
    
    step_number += 1
    realtime_timer.Spin(step_size)