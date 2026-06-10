"""
FEDA double lane-change maneuver with path-follower driver.

Vehicle: FEDA on rigid flat terrain.
Driver: ChPathFollowerDriver on ISO double-lane-change path.
System: NSC (ChContactMethod_NSC), Bullet collision.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters ===
STEERING_KP = 0.8
STEERING_KI = 0.0
STEERING_KD = 0.0
STEERING_LOOK_AHEAD = 5.0

SPEED_KP = 0.4
SPEED_KI = 0.0
SPEED_KD = 0.0
TARGET_SPEED = 10.0

TIME_STEP = 1e-3
SIM_END = 20.0
RENDER_FPS = 50.0

VEHICLE_INIT = chrono.ChVector3d(-50.0, 0.0, 0.5)
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 10.0

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle (FEDA — NSC, rigid terrain) ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)
feda.SetInitPosition(chrono.ChCoordsysd(VEHICLE_INIT, chrono.QUNIT))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(TIME_STEP)
feda.Initialize()
system = feda.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

# === Terrain (RigidTerrain — flat NSC patch) ===
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
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Path-follower driver ===
start = chrono.ChVector3d(-50.0, 0.0, 0.5)
path = veh.DoubleLaneChangePath(start, 13.5, 4.0, 11.0, 50.0, True)

driver = veh.ChPathFollowerDriver(
    feda.GetVehicle(),
    path,
    "double_lane_change",
    TARGET_SPEED,
)
driver.GetSteeringController().SetLookAheadDistance(STEERING_LOOK_AHEAD)
driver.GetSteeringController().SetGains(STEERING_KP, STEERING_KI, STEERING_KD)
driver.GetSpeedController().SetGains(SPEED_KP, SPEED_KI, SPEED_KD)
driver.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA — Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(feda.GetVehicle())

# === Review-only setup ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))

# === Simulation loop ===
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

os.makedirs("frames", exist_ok=True)

# review-only: CSV logging
csv_path = "simulation_data.csv"
csv_file = None
csv_writer = None
if REC:
    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=["time", "x", "y", "z", "speed"])
    csv_writer.writeheader()

frame = 0
while vis.Run() and system.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        feda.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        feda.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

        # review-only: log CSV
        if REC and csv_writer is not None:
            chassis = feda.GetChassisBody()
            pos = chassis.GetPos()
            speed = feda.GetVehicle().GetSpeed()
            csv_writer.writerow({
                "time": sim_time,
                "x": pos.x, "y": pos.y, "z": pos.z,
                "speed": speed,
            })

        if system.GetChTime() >= SIM_END:
            break

    # review-only: capture frame
    if REC:
        vis.WriteImageToFile(f"frames/img_{frame:06d}.png")
    frame += 1

# review-only: close CSV, assemble video
if REC:
    if csv_file is not None:
        csv_file.close()
    import sim_recording as rec
    irr_dir = rec.frame_dir("frames")
    rec.assemble_all_videos(irr_dir)
    rec.cleanup_frames(irr_dir)
