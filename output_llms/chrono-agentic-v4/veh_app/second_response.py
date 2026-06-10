"""
SimBench veh_app turn 2 — HMMWV with lidar sensor, box, and cylinder.

Vehicle: HMMWV_Full on flat rigid terrain.
Objects: Box (1x1x1 at z=0.5) and cylinder (r=0.5, h=1 at z=1.5), blue texture.
Sensor: Lidar attached to chassis, offset (0, 0, 2).
Driver: Scripted inputs (steering=0.5, throttle=0.2).
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants ===
TIME_STEP = 1e-3
SIM_END = 20.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Vehicle init position — input2 changes initLoc from (0,0,0.4) to (0,-5,0.4)
INIT_LOC = chrono.ChVector3d(0, -5, 0.4)
INIT_ROT = chrono.QUNIT

# Box: 1x1x1 at (0, 0, 0.5)
BOX_POS = chrono.ChVector3d(0, 0, 0.5)
BOX_SIZE = chrono.ChVector3d(1, 1, 1)

# Cylinder: r=0.5, h=1 at (0, 0, 1.5)
CYL_POS = chrono.ChVector3d(0, 0, 1.5)
CYL_RAD = 0.5
CYL_H = 1.0

# Lidar offset on chassis: (0, 0, 2)
LIDAR_OFFSET = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 2),
    chrono.QUNIT,
)

# === System & collision ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain (rigid flat) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()

# === Scene objects: box and cylinder with blue texture ===
# Box at (0, 0, 0.5)
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.5)
box = chrono.ChBodyEasyBox(BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, 1000.0, True, True, box_mat)
box.SetPos(BOX_POS)
box.SetFixed(False)
system.AddBody(box)

# Cylinder at (0, 0, 1.5)
cyl_mat = chrono.ChContactMaterialNSC()
cyl_mat.SetFriction(0.5)
cyl = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, CYL_RAD, CYL_H, 1000.0, True, True, cyl_mat)
cyl.SetPos(CYL_POS)
cyl.SetFixed(False)
system.AddBody(cyl)

# Blue color for box and cylinder
blue_color = chrono.ChColor(0.2, 0.4, 0.8)
try:
    box.GetVisualShape(0).SetColor(blue_color)
except (AttributeError, IndexError):
    pass
try:
    cyl.GetVisualShape(0).SetColor(blue_color)
except (AttributeError, IndexError):
    pass

# === Lidar sensor ===
manager = sens.ChSensorManager(system)
# Lidar is not a camera — no scene lights required per sensor_manager skill

lidar = sens.ChLidarSensor(
    hmmwv.GetChassisBody(),   # attached to chassis
    5.0,                       # update_rate Hz
    LIDAR_OFFSET,
    800,                       # horizontal_samples
    300,                       # vertical_samples
    2 * chrono.CH_PI,          # horizontal_fov
    chrono.CH_PI / 12,         # max_vert_angle
    -chrono.CH_PI / 6,        # min_vert_angle
    100.0,                     # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                         # sample_radius
    0.003,                     # vert divergence_angle
    0.003,                     # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)

# Lidar filter chain
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("SimBench veh_app turn 2")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(RENDER_EVERY * TIME_STEP / steering_time)
driver.SetThrottleDelta(RENDER_EVERY * TIME_STEP / throttle_time)
driver.SetBrakingDelta(RENDER_EVERY * TIME_STEP / braking_time)
driver.Initialize()

# === CSV logging ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))
irr_dir = None
if REC:
    import sim_recording as rec
    irr_dir = rec.frame_dir("frames")

os.makedirs("cam", exist_ok=True)
csv_path = "simulation_data.csv"
csv_file = None
csv_writer = None
if REC and csv_path:
    csv_file = open(csv_path, "w", newline="")
    csv_writer = __import__('csv').writer(csv_file)
    csv_writer.writerow(["time", "chassis_x", "chassis_y", "chassis_z", "speed"])

# === Main loop ===
frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        if REC and irr_dir:
            vis.WriteImageToFile(f"{irr_dir}/img_{frame:06d}.png")

        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()

            # --- Scripted driver inputs per input2.txt: steering=0.5, throttle=0.2 ---
            driver_inputs = driver.GetInputs()
            driver_inputs.m_steering = 0.5
            driver_inputs.m_throttle = 0.2
            driver_inputs.m_braking = 0.0

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)
            manager.Update()

            # CSV logging
            if REC and csv_writer:
                chassis_pos = hmmwv.GetChassisBody().GetPos()
                speed = hmmwv.GetVehicle().GetSpeed()
                csv_writer.writerow([
                    sim_time,
                    chassis_pos.x, chassis_pos.y, chassis_pos.z,
                    speed
                ])

            step_number += 1
            if system.GetChTime() >= SIM_END:
                break

        frame += 1
        realtime_timer.Spin(RENDER_EVERY * TIME_STEP)

finally:
    if REC:
        if csv_file:
            csv_file.close()
        if irr_dir:
            import sim_recording as rec
            rec.assemble_all_videos(irr_dir, sensor_dirs=["cam/lidar"])
            rec.cleanup_frames(irr_dir, "cam/lidar")

# === Post-processing ===
if REC and os.path.exists("simulation_data.csv"):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import csv as csv_lib

        times, xs, ys, zs, speeds = [], [], [], [], []
        with open("simulation_data.csv", "r") as f:
            reader = csv_lib.DictReader(f)
            for row in reader:
                times.append(float(row["time"]))
                xs.append(float(row["chassis_x"]))
                ys.append(float(row["chassis_y"]))
                zs.append(float(row["chassis_z"]))
                speeds.append(float(row["speed"]))

        fig, axes = plt.subplots(2, 1, figsize=(10, 8))
        axes[0].plot(times, xs, label="X"); axes[0].plot(times, ys, label="Y")
        axes[0].plot(times, zs, label="Z"); axes[0].set_xlabel("Time (s)")
        axes[0].set_ylabel("Position (m)"); axes[0].legend(); axes[0].grid(True)
        axes[1].plot(times, speeds, color="green")
        axes[1].set_xlabel("Time (s)"); axes[1].set_ylabel("Speed (m/s)")
        axes[1].grid(True)
        plt.tight_layout()
        plt.savefig("simulation_timeseries.png", dpi=100)
        print("Saved simulation_timeseries.png")
    except Exception as exc:
        import traceback; traceback.print_exc()
