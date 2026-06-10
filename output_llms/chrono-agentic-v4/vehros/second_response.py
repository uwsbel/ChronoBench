"""
SimBench vehros turn=2 — HMMWV vehicle with terrain and Irrlicht visualization.

This simulation models an HMMWV full-model vehicle driving on a rigid terrain patch.
The scene includes enhanced visualization settings for all vehicle subsystems,
terrain texturing, and a real-time Irrlicht render loop.

System type: NSC (ChContactMethod_NSC) with Bullet collision.
"""

import os
import math
import csv
import numpy as np
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import sim_recording as rec  # scored core: used in frame capture

# === Named constants ===
TIME_STEP = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# HMMWV spawn position (Z derived from terrain height + suspension ref height)
HMMWV_INIT_X = 0.0
HMMWV_INIT_Y = 0.0
HMMWV_INIT_Z = 1.0
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0

REC = bool(os.environ.get("SIMBENCH_RECORD"))  # scored core: controls frame capture guard
irr_dir = rec.frame_dir("frames") if REC else None  # scored core: used in frame capture


# === System & gravity ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(
        chrono.ChVector3d(HMMWV_INIT_X, HMMWV_INIT_Y, HMMWV_INIT_Z),
        chrono.QUNIT,
    )
)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === System bodies (created by the veh.HMMWV_Full wrapper) ===
chassis = hmmwv.GetChassisBody()  # cache: main chassis rigid body
# terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper

# === Terrain (rigid flat) ===
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
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Enhanced visualization settings (applied after Initialize) ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Visualization — full Irrlicht scene ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Vehros — HMMWV on Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(TIME_STEP / steering_time)
driver.SetThrottleDelta(TIME_STEP / throttle_time)
driver.SetBrakingDelta(TIME_STEP / braking_time)
driver.Initialize()

# === CSV logging ===
csv_path = "simulation_data.csv"
csv_file = None
csv_writer = None
try:
    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "time", "pos_x", "pos_y", "pos_z",
        "vel_x", "vel_y", "vel_z",
        "steering", "throttle", "braking",
    ])
finally:
    pass  # opened inside try; closed in finally below


# === Main loop ===
frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run() and system.GetChTime() < SIM_END:
    # Throttled rendering
    if step_number % RENDER_EVERY == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        if REC:
            vis.WriteImageToFile(rec.frame_path(irr_dir, frame))
            frame += 1

    sim_time = system.GetChTime()
    driver_inputs = driver.GetInputs()

    # review-only: drive open-loop for the validation video

    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    hmmwv.Synchronize(sim_time, driver_inputs, terrain)
    vis.Synchronize(sim_time, driver_inputs)

    driver.Advance(TIME_STEP)
    terrain.Advance(TIME_STEP)
    hmmwv.Advance(TIME_STEP)
    vis.Advance(TIME_STEP)

    # Log CSV every physics step
    if csv_writer is not None:
        chassis_pos = chassis.GetPos()
        chassis_vel = chassis.GetLinVel()
        csv_writer.writerow([
            sim_time,
            chassis_pos.x, chassis_pos.y, chassis_pos.z,
            chassis_vel.x, chassis_vel.y, chassis_vel.z,
            driver_inputs.m_steering,
            driver_inputs.m_throttle,
            driver_inputs.m_braking,
        ])

    step_number += 1
    realtime_timer.Spin(TIME_STEP)

# === Close CSV ===
if csv_file is not None:
    csv_file.close()

# === Review-only post-processing ===
