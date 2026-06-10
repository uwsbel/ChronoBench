"""
CityBus simulation with data-driven driver system.

plan_type: mbs_in_scene
vehicle: CityBus with ChDataDriver (programmatic throttle/steering/brake schedule)
terrain: RigidTerrain (flat NSC)
objective: demonstrate ChDataDriver with a piecewise driver-input schedule
"""

import os
import math
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Review-only: sim_recording for video capture ===

# -------------------------------------------------------------------------
# 1. Named constants
# -------------------------------------------------------------------------
STEP_SIZE = 5e-3          # physics time-step (s)
SIM_END   = 12.0          # total simulation duration (s)
RENDER_FPS = 50.0          # display/video frame rate
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))

# -------------------------------------------------------------------------
# 2. System + gravity
# -------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")

bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(STEP_SIZE)
bus.Initialize()

sys = bus.GetSystem()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# === System & bodies (created by the veh.CityBus wrapper) ===
# sys          : ChSystemNSC owned by the wrapper
# chassis      : main bus body
# wheels       : via bus.GetVehicle().GetAxles()
# terrain      : RigidTerrain patch below
# driver       : ChDataDriver (data-driven schedule)
# vis          : ChWheeledVehicleVisualSystemIrrlicht

# -------------------------------------------------------------------------
# 3. Terrain — RigidTerrain with NSC patch
# -------------------------------------------------------------------------
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    200.0,   # length (m)
    200.0,   # width (m)
)
terrain.Initialize()
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)

# -------------------------------------------------------------------------
# 4. Driver — ChDataDriver with piecewise schedule
#    At 0.0s: throttle=0.0, steering=0.0, braking=0.0
#    At 0.1s: throttle=1.0, steering=0.0, braking=0.0
#    At 0.5s: throttle=1.0, steering=0.7, braking=0.0
# -------------------------------------------------------------------------
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0),
    veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0),
])
driver = veh.ChDataDriver(bus.GetVehicle(), driver_data)
driver.Initialize()

# -------------------------------------------------------------------------
# 5. Visualization — Irrlicht
# -------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus — ChDataDriver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.5), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(bus.GetVehicle())

# -------------------------------------------------------------------------
# 6. Review-only: CSV data logging
# -------------------------------------------------------------------------
csv_file = None
data_writer = None
if REC:  # review-only >>>
    os.makedirs("cam", exist_ok=True)   # CSV + mp4 go here
    os.makedirs("frames", exist_ok=True)  # PNG frames go here for cleanup
    csv_path = "cam/simulation_data.csv"
    csv_file = open(csv_path, "w", newline="")
    data_writer = csv.writer(csv_file)
    data_writer.writerow([
        "time", "chassis_x", "chassis_y", "chassis_z",
        "chassis_vx", "chassis_vy", "chassis_vz",
        "speed", "steering", "throttle", "braking",
    ])

# -------------------------------------------------------------------------
# 7. Main loop — Synchronize/Advance order: driver -> terrain -> vehicle -> vis
# -------------------------------------------------------------------------
frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
sim_time = sys.GetChTime()

while vis.Run() and sim_time < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    if REC and frame % render_every == 0:  # review-only >>>
        try:
            vis.WriteImageToFile(rec.frame_path("frames", frame))
        except Exception as ex:
            print(f"[capture] WriteImageToFile failed: {ex}")

    for _ in range(render_every):
        sim_time = sys.GetChTime()

        driver_inputs = driver.GetInputs()
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        bus.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        bus.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        if REC and data_writer is not None:  # review-only >>>
            chassis = bus.GetChassisBody()
            v = chassis.GetPos()
            vel = chassis.GetLinVel()
            speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            data_writer.writerow([
                f"{sim_time:.4f}",
                f"{v.x:.4f}", f"{v.y:.4f}", f"{v.z:.4f}",
                f"{vel.x:.4f}", f"{vel.y:.4f}", f"{vel.z:.4f}",
                f"{speed:.4f}",
                f"{driver_inputs.m_steering:.4f}",
                f"{driver_inputs.m_throttle:.4f}",
                f"{driver_inputs.m_braking:.4f}",
            ])

        if sim_time >= SIM_END:
            break

    frame += 1
    realtime_timer.Spin(STEP_SIZE)
    sim_time = sys.GetChTime()

# -------------------------------------------------------------------------
# 8. Review-only: close CSV, assemble video, plot time-series
# -------------------------------------------------------------------------
# Close CSV
if csv_file is not None:
    csv_file.flush()
    csv_file.close()
    print(f"[DEBUG] CSV closed, exists={os.path.exists('cam/simulation_data.csv')}, size={os.path.getsize('cam/simulation_data.csv')}")

# Assemble video from captured frames
if REC:  # review-only >>>
    try:
        rec.assemble_all_videos("frames", sensor_dirs=[])
        rec.cleanup_frames("frames")
    except Exception as ex:
        print(f"[video assembly] failed: {ex}")

    # Plot time-series from CSV
    try:
        times = []
        speeds = []
        steerings = []
        throttle_vals = []
        brake_vals = []
        with open("cam/simulation_data.csv", "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row["time"]))
                speeds.append(float(row["speed"]))
                steerings.append(float(row["steering"]))
                throttle_vals.append(float(row["throttle"]))
                brake_vals.append(float(row["braking"]))

        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        axes[0, 0].plot(times, speeds)
        axes[0, 0].set_xlabel("Time (s)")
        axes[0, 0].set_ylabel("Speed (m/s)")
        axes[0, 0].set_title("Chassis Speed")

        axes[0, 1].plot(times, steerings)
        axes[0, 1].set_xlabel("Time (s)")
        axes[0, 1].set_ylabel("Steering")
        axes[0, 1].set_title("Steering Input")

        axes[1, 0].plot(times, throttle_vals)
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_ylabel("Throttle")
        axes[1, 0].set_title("Throttle Input")

        axes[1, 1].plot(times, brake_vals)
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_ylabel("Braking")
        axes[1, 1].set_title("Braking Input")

        plt.tight_layout()
        plt.savefig("cam/simulation_timeseries.png", dpi=150)
        plt.close()
    except Exception as ex:
        import traceback
        traceback.print_exc()
        print(f"[plotting] failed: {ex}")
