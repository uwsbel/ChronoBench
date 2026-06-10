"""
SCM Hill with HMMWV — Turn 2
Additions over Turn 1: 5 randomly-positioned box obstacles, lidar sensor on vehicle.
plan_type: mbs_in_scene
System: SMC (required for SCM terrain)
"""

import os
import math
import csv
import numpy as np

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Simulation parameters ===
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 20.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))
sim_end = 30.0

# === Vehicle init ===
initLoc = chrono.ChVector3d(-15, 0, 1.2)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
contact_method = chrono.ChContactMethod_SMC

vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === SCM Terrain ===
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    2e6,   # Bekker Kphi
    0,     # Bekker Kc
    1.1,   # Bekker n exponent
    0,     # Mohr cohesive limit (Pa)
    30,    # Mohr friction limit (degrees)
    0.01,  # Janosi shear coefficient (m)
    2e8,   # Elastic stiffness (Pa/m)
    3e4,   # Damping (Pa s/m)
)
terrain.AddMovingPatch(
    vehicle.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    40, 40, -1, 1, 0.02,
)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# === Box obstacles (5 randomly positioned) ===
rng = np.random.default_rng(42)
obs_size = 1.0
obs_density = 500.0
obs_mat = chrono.ChContactMaterialSMC()
obs_mat.SetFriction(0.8)
obs_mat.SetRestitution(0.1)

for i in range(5):
    ox = rng.uniform(-20, 20)
    oy = rng.uniform(-5, 5)
    oh = rng.uniform(0.5, 2.5)
    obs_body = chrono.ChBodyEasyBox(
        obs_size, obs_size, obs_size,
        obs_density, True, True, obs_mat,
    )
    obs_body.SetPos(chrono.ChVector3d(ox, oy, obs_size / 2.0 + oh))
    obs_body.SetFixed(False)
    vehicle.GetSystem().AddBody(obs_body)

# === Sensor Manager + Lidar ===
manager = sens.ChSensorManager(vehicle.GetSystem())
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)

# Lidar attached to chassis
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),
    5.0,                               # update_rate Hz
    lidar_offset,
    800,                               # horizontal samples
    1,                                 # vertical samples (2D lidar)
    2 * chrono.CH_PI,                  # horizontal fov
    0,                                 # max_vert_angle
    0,                                 # min_vert_angle
    100.0,                             # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Irrlicht visualization ===
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV SCM Hill — Turn 2")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Review-only recording setup ===


frame = 0

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        sim_time = vehicle.GetSystem().GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        vehicle.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        manager.Update()


        step_number += 1
        if vehicle.GetSystem().GetChTime() >= sim_end:
            break

    realtime_timer.Spin(step_size)
