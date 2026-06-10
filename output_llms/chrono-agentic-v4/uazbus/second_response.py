"""
UAZBUS double-lane-change simulation.
plan_type: mbs_in_scene (wrapper-managed wheeled vehicle)
System: ChSystemNSC with UAZB catalog vehicle on rigid terrain.
Maneuver: ISO double lane change at constant speed via ChPathFollowerDriver.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
time_step = 1e-3
sim_end = 30.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Vehicle initialization (turn 2 delta: position = (-40, 0, 0.5)) ===
init_loc = chrono.ChVector3d(-40, 0, 0.5)
init_rot = chrono.QUNIT

uazb = veh.UAZBUS()
uazb.SetContactMethod(chrono.ChContactMethod_NSC)
uazb.SetChassisCollisionType(veh.CollisionType_NONE)
uazb.SetChassisFixed(False)
uazb.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
uazb.SetTireType(veh.TireModelType_TMEASY)
uazb.SetTireStepSize(time_step)
uazb.Initialize()

# Set visualization types to MESH so the vehicle body, wheels, and tires are visible.
uazb.SetChassisVisualizationType(veh.VisualizationType_MESH)
uazb.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
uazb.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
uazb.SetWheelVisualizationType(veh.VisualizationType_MESH)
uazb.SetTireVisualizationType(veh.VisualizationType_MESH)

system = uazb.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", uazb.GetVehicle().GetMass())

# === Terrain: rigid with concrete texture (turn 2 delta: tile4 → concrete) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    600.0,
    600.0,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 300, 300)
terrain.Initialize()

# === Double lane change path + path-follower driver ===
# Start near the vehicle initial position so the path begins where the vehicle spawns.
path_start = chrono.ChVector3d(-40.0, 0.0, 0.5)
lane_path = veh.DoubleLaneChangePath(
    path_start,     # start position
    13.5,           # length of DLC segment
    4.0,            # lane width
    11.0,           # offset (total lateral displacement)
    50.0,           # total path length
    True,           # to left first
)

target_speed = 8.0  # m/s
driver = veh.ChPathFollowerDriver(
    uazb.GetVehicle(),
    lane_path,
    "double_lane_change",
    target_speed,
)
driver.GetSteeringController().SetLookAheadDistance(5.0)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)
driver.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(uazb.GetVehicle())

os.makedirs("cam", exist_ok=True)

# === Review-only recording scaffolding ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and uazb.GetSystem().GetChTime() < sim_end:
    sim_time = uazb.GetSystem().GetChTime()

    if step_number % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()
    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    uazb.Synchronize(sim_time, driver_inputs, terrain)
    vis.Synchronize(sim_time, driver_inputs)

    driver.Advance(time_step)
    terrain.Advance(time_step)
    uazb.Advance(time_step)
    vis.Advance(time_step)

    step_number += 1
    realtime_timer.Spin(time_step)
