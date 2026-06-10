"""
ARTcar demo — wheeled vehicle on rigid terrain.
Applies deltas from input2: init location (1,0,0.5), PRIMITIVES viz,
MESH chassis collision, FIALA tire model.
"""
import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Simulation parameters ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Initial vehicle location and orientation — delta from input2
init_loc = chrono.ChVector3d(1, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type — delta: PRIMITIVES (not MESH)
vis_type = veh.VisualizationType_PRIMITIVES

# Chassis collision type — delta: MESH (not NONE)
chassis_collision_type = veh.CollisionType_MESH

# Tire model type — delta: FIALA (not TMEASY)
tire_model = veh.TireModelType_FIALA

# Terrain
terrain_height = 0.0
terrain_length = 100.0
terrain_width = 100.0

# Camera chase point
track_point = chrono.ChVector3d(0.0, 0.0, 0.2)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# === Create ARTcar vehicle ===
vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(time_step)
vehicle.SetMaxMotorVoltageRatio(0.16)
vehicle.SetStallTorque(0.3)
vehicle.SetTireRollingResistance(0.06)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# === System & collision (REQUIRED after Initialize for contact scenes) ===
system = vehicle.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('ARTcar — turn 2')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(track_point, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_fps / steering_time)
driver.SetThrottleDelta(render_fps / throttle_time)
driver.SetBrakingDelta(render_fps / braking_time)
driver.Initialize()

# === Diagnostic ===
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Review-only recording ===

# === Main loop ===
frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        vehicle.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)
        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        vis.Advance(time_step)
        step_number += 1
        if system.GetChTime() >= sim_end:
            break
    realtime_timer.Spin(time_step)

# === Review-only post-processing ===
