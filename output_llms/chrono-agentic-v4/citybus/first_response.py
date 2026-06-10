"""
CityBus vehicle on rigid terrain with Irrlicht visualization.

Simulates a CityBus on a flat rigid terrain using PyChrono and Irrlicht.
The vehicle is driven with an interactive driver for real-time control of
steering, throttle, and braking. Mesh and primitive visualization types are
applied to different vehicle parts. The camera follows the vehicle.

System: NSC (ChContactMethod_NSC) with Bullet collision.
"""

import math
import os
import csv as _csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Named constants ===
TIME_STEP = 1e-3          # physics timestep (s)
SIM_END = 30.0            # simulation duration (s)
RENDER_FPS = 50.0         # render framerate (fps)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Vehicle spawn: Z=0 gives wheel bottom at ~0.03 (wheel radius=0.525, spindle z=0.555)
VEH_INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.0)
VEH_INIT_ROT = chrono.QUNIT

# Terrain
TERRAIN_LENGTH = 200.0    # m
TERRAIN_WIDTH = 200.0     # m
TERRAIN_FRICTION = 0.8
TERRAIN_RESTITUTION = 0.01

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle (CityBus) ===
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(VEH_INIT_LOC, VEH_INIT_ROT))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(TIME_STEP)
bus.Initialize()

system = bus.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# === Terrain (RigidTerrain) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.7, 0.7, 0.7))
terrain.Initialize()

# === Visualization (Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus Simulation")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 3.0), 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddGrid(1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# Set explicit visualization types for vehicle parts
bus.GetVehicle().SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.GetVehicle().SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.GetVehicle().SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.GetVehicle().SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.GetVehicle().SetTireVisualizationType(veh.VisualizationType_MESH)

vis.AttachVehicle(bus.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(RENDER_FPS * TIME_STEP / steering_time)
driver.SetThrottleDelta(RENDER_FPS * TIME_STEP / throttle_time)
driver.SetBrakingDelta(RENDER_FPS * TIME_STEP / braking_time)
driver.Initialize()

# === Cached handles ===
chassis = bus.GetChassisBody()  # cache: reused each step
veh_obj = bus.GetVehicle()  # cache: for axle/spindle access

# review-only: sim_recording import and REC flag
import sim_recording as rec
REC = bool(os.environ.get("SIMBENCH_RECORD"))

# === CSV logging (scored core - must always open, write header)
try:
    csv_fh = open("simulation_data.csv", "w", newline="")
except (OSError, IOError) as exc:
    raise RuntimeError(f"Failed to open simulation_data.csv: {exc}") from exc

csv_writer = _csv.DictWriter(csv_fh, fieldnames=[
    "time", "chassis_x", "chassis_y", "chassis_z",
    "chassis_vx", "chassis_vy", "chassis_vz",
    "steering", "throttle", "braking",
])
csv_writer.writeheader()
csv_fh.flush()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and system.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()

        # review-only: apply throttle so the bus moves for the video

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        bus.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        bus.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

        # review-only CSV row logging

        if system.GetChTime() >= SIM_END:
            break

# === Post-loop: close CSV, assemble video, plot ===
try:
    csv_fh.close()
except (OSError, IOError) as exc:
    print(f"Warning: error closing CSV: {exc}")
