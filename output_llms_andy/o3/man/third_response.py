# =============================================================================
# MAN 10 t demo
#  - added Chrono::Sensor lidar
#  - added several randomly-sized/positioned dynamic boxes
#  - changed the ground texture to grass.jpg
#  - sensor manager is updated inside the simulation loop
# =============================================================================
import math
import random

import numpy as np                            # new
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens                # new


# -----------------------------------------------------------------------------
# Chrono data paths
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
sens.SetChronoDataPath(chrono.GetChronoDataPath())

# -----------------------------------------------------------------------------
# Initial vehicle location and orientation
# -----------------------------------------------------------------------------
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# -----------------------------------------------------------------------------
# General settings
# -----------------------------------------------------------------------------
vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.ChassisCollisionType_NONE      # fixed typo
tire_model             = veh.TireModelType_TMEASY
contact_method         = chrono.ChContactMethod_NSC
step_size              = 1e-3
render_step_size       = 1.0 / 50.0                         # 50 FPS
terrainHeight          = 0.0
terrainLength          = 100.0
terrainWidth           = 100.0
trackPoint             = chrono.ChVector3d(-3.0, 0.0, 1.1)

# -----------------------------------------------------------------------------
# Create the vehicle
# -----------------------------------------------------------------------------
vehicle = veh.MAN_10t()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# make sure we have Bullet so the sensor ray-casting works fast
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -----------------------------------------------------------------------------
# Create the terrain
# -----------------------------------------------------------------------------
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
# changed the texture here ----------------------------------------------------
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# -----------------------------------------------------------------------------
# Irrlicht visualization
# -----------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo with Lidar')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# -----------------------------------------------------------------------------
# Interactive driver
# -----------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)
steering_time, throttle_time, braking_time = 1.0, 1.0, 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# -----------------------------------------------------------------------------
# (NEW)  Create a sensor manager and a lidar sensor
# -----------------------------------------------------------------------------
manager = sens.ChSensorManager(vehicle.GetSystem())

# Place the lidar on top of the cab (local frame)
lidar_offset = chrono.ChFrameD(chrono.ChVector3d(0.0, 0.0, 1.8), chrono.QUNIT)

lidar_update_rate   = 10.0                       # Hz
h_samples, v_samples = 1024, 32
h_fov, v_fov        = math.radians(90), math.radians(30)
lidar_max_range     = 100.0                      # m

lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),       # parent body
    lidar_update_rate,              # scanning rate
    lidar_offset,                   # offset pose
    h_samples,
    v_samples,
    h_fov,
    v_fov,
    lidar_max_range,
)
lidar.SetName("Lidar Sensor")

# Store raw point-clouds as PCD files (could be omitted)
lidar.PushFilter(sens.ChFilterPCD("lidar_output/"))

manager.AddSensor(lidar)

# -----------------------------------------------------------------------------
# (NEW)  Add a handful of random dynamic boxes to have obstacles for the lidar
# -----------------------------------------------------------------------------
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.8)

rand = random.Random(12345)
num_boxes = 20
for _ in range(num_boxes):
    sx = rand.uniform(0.3, 1.0)
    sy = rand.uniform(0.3, 1.0)
    sz = rand.uniform(0.3, 1.0)
    density = 800          # kg/m^3 (just any reasonable value)

    box = chrono.ChBodyEasyBox(sx, sy, sz,          # size (full lengths)
                               density,
                               True,                # visualization asset
                               True,                # collision
                               box_mat)

    px = rand.uniform(-terrainLength / 2.0, terrainLength / 2.0)
    py = rand.uniform(-terrainWidth  / 2.0, terrainWidth  / 2.0)
    pz = terrainHeight + sz / 2.0 + 0.01            # sit on ground

    box.SetPos(chrono.ChVector3d(px, py, pz))
    box.SetBodyFixed(False)
    vehicle.GetSystem().Add(box)

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
render_steps   = math.ceil(render_step_size / step_size)
step_number    = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # 1. Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # 2. Driver inputs
    driver_inputs = driver.GetInputs()

    # 3. Module synchronization
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # 4. Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # 5. (NEW) Update the sensor manager AFTER dynamics step
    manager.Update()

    # 6. Maintain realtime pace
    realtime_timer.Spin(step_size)

    # 7. Book-keeping
    step_number += 1