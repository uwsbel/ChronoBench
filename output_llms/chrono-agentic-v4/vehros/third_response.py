"""
 vehros turn 3 — HMMWV on RigidTerrain with ROS bridge and Lidar sensor.

 Modifications from turn 2 (isolated — reconstructing full scene):
   1. Added `import pychrono.sensor as sens` for sensor functionality.
   2. Added a visualization box using `ChBodyEasyBox`.
   3. Set up `ChSensorManager` to manage sensors.
   4. Added and configured a `ChLidarSensor` with various filters.
   5. Registered `ChROSLidarHandler` to publish lidar data to ROS.
   6. Updated sensor manager within the simulation loop using `sens_manager.Update()`.
   7. Changed camera position to `(-5, 2.5, 1.5)` for a new perspective.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as chros

# review-only: sim_recording scaffolding

# === Named constants ===
TIME_STEP = 1e-3
SIM_END = 20.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0

VEH_INIT_X = 0.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = 0.5
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3


# === System & gravity ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z),
    chrono.QUNIT,
))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain (RigidTerrain with NSC material) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization box (turn 3 addition — ChBodyEasyBox) ===
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.0)
viz_box = chrono.ChBodyEasyBox(
    1.0,    # X length
    1.0,    # Y length
    1.0,    # Z length
    1000.0,  # density
    True,    # visualize
    True,    # collide
    box_mat,
)
viz_box.SetPos(chrono.ChVector3d(5.0, 0.0, 0.5))
viz_box.SetFixed(False)
viz_box.SetName("viz_box")
system.AddBody(viz_box)

# === ROS bridge (ChROSPythonManager) ===
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Body handler — publishes chassis pose (before driver is created)
chassis_body = hmmwv.GetChassisBody()
ros_body_handler = chros.ChROSBodyHandler(30, chassis_body, "~/output/chassis")
ros_manager.RegisterHandler(ros_body_handler)

# === Sensor manager (turn 3 addition) ===
sens_manager = sens.ChSensorManager(system)

# Point light for camera/lidar rendering
sens_manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)

# Lidar sensor — attached to the chassis body
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-1.0, 0.0, 0.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis_body,
    5.0,                               # update_rate Hz
    lidar_offset,
    800,                               # horizontal samples
    1,                                 # vertical samples (2D lidar)
    2 * chrono.CH_PI,                  # horizontal fov
    0.0,                               # max_vert_angle
    0.0,                               # min_vert_angle
    100.0,                             # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                                 # sample_radius
    0.003,                             # vert divergence_angle
    0.003,                             # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)

# Lidar filter chain
lidar.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())

# ChROSLidarHandler — registered after sensor filters are pushed
ros_lidar_handler = chros.ChROSLidarHandler(lidar, "~/output/lidar")
ros_manager.RegisterHandler(ros_lidar_handler)

sens_manager.AddSensor(lidar)

# === Irrlicht visualization (window + sky + camera + lights) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle(" vehros — turn 3")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Camera position update (turn 3: new perspective) ===
vis.AddCamera(
    chrono.ChVector3d(-5, 2.5, 1.5),   # turn 3: new camera position
    chrono.ChVector3d(0, 0, 0),
)

# === Driver (created AFTER vis is built — ChInteractiveDriverIRR takes vis) ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(1.0 / STEERING_TIME / RENDER_FPS)
driver.SetThrottleDelta(1.0 / THROTTLE_TIME / RENDER_FPS)
driver.SetBrakingDelta(1.0 / BRAKING_TIME / RENDER_FPS)
driver.Initialize()

ros_manager.Initialize()


# === Main loop ===
frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run() and system.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(RENDER_EVERY):
        sim_time = system.GetChTime()

        # Sensor manager update (turn 3 addition — every physics step)
        sens_manager.Update()

        # ROS update
        if not ros_manager.Update(sim_time, TIME_STEP):
            break

        driver_inputs = driver.GetInputs()
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)


        if system.GetChTime() >= SIM_END:
            break

        step_number += 1
        realtime_timer.Spin(TIME_STEP)
