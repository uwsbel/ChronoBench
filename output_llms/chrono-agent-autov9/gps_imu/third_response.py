"""GPS + IMU sensing on a driving HMMWV (PyChrono 9.0.1, Irrlicht renderer).

Model
-----
A full HMMWV wheeled vehicle (veh.HMMWV_Full wrapper, SMC contact) drives across
a flat rigid terrain patch. Three sensors are rigidly mounted on the chassis via a
ChSensorManager:
  * a 3-axis accelerometer (ChAccelerometerSensor),
  * a 3-axis gyroscope    (ChGyroscopeSensor),  -> together the IMU,
  * a GPS receiver        (ChGPSSensor) referenced to a geodetic origin.
All three share the same chassis-relative offset pose of (0, 0, 1) m (one metre
above the chassis origin, no rotation).

System type
-----------
SMC (penalty contact) — the HMMWV_Full wrapper owns a ChSystemSMC internally; the
rigid terrain patch and the sensor manager attach to that same owned system.

Driver / expected behavior
--------------------------
The driver holds a CONSTANT steering of 0.6 and a CONSTANT throttle of 0.5 for the
entire run, so the vehicle accelerates forward while turning — tracing a curved
(roughly circular/arc) ground path. The accelerometer should show a non-zero
forward + centripetal signal once moving, the gyroscope a non-zero yaw rate, and
the GPS a smoothly varying latitude/longitude track. A matplotlib figure plots the
GPS trajectory in the longitude (x) / latitude (y) plane at the end of the run.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")                       # headless-safe backend for PNG output
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants (geometry / physics / control) — no bare literals downstream ===
TIME_STEP = 2.0e-3                           # integration step (s)
TIRE_STEP = 1.0e-3                           # tire substep (s)
SIM_END = 12.0                               # simulation duration (s)
RENDER_FPS = 30.0                            # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

CONST_STEERING = 0.6                         # prompt: constant steering held all run
CONST_THROTTLE = 0.5                         # prompt: constant throttle held all run
CONST_BRAKING = 0.0

VEH_INIT_X = 0.0                             # spawn X (m), geometric-center origin
VEH_INIT_Y = 0.0                             # spawn Y (m)
SUSPENSION_REF_HEIGHT = 0.5                  # HMMWV chassis origin above wheel-bottom
TERRAIN_TOP_Z = 0.0                          # flat rigid patch top surface (m)
VEH_INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT
TIRE_RADIUS = 0.46                           # HMMWV tire radius (m), for footprint check
ZTOL = 0.10                                  # allowed wheel-bottom clearance vs support

TERRAIN_LENGTH = 200.0                       # rigid patch X extent (m)
TERRAIN_WIDTH = 200.0                        # rigid patch Y extent (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_YOUNG = 2.0e7                        # SMC stiffness (Pa)

# Sensor mounting + rates.
IMU_OFFSET = chrono.ChVector3d(0, 0, 1)      # prompt: IMU offset pose (0, 0, 1) m
GPS_OFFSET = chrono.ChVector3d(0, 0, 1)      # GPS shares the same chassis offset
IMU_UPDATE_RATE = 100.0                      # Hz
GPS_UPDATE_RATE = 10.0                       # Hz
GPS_REFERENCE = chrono.ChVector3d(43.073268, -89.400636, 260.0)  # (lat, lon, alt) origin

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast windowless validation gate
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
# The wrapper instantiates a ChSystemSMC plus the chassis, four spindles, the
# suspension/steering joints, engine, transmission, and tires internally.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT)
)
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # slip-curve tire so the vehicle actually drives
hmmwv.SetTireStepSize(TIRE_STEP)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

system = hmmwv.GetSystem()                    # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()              # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()                  # cache: vehicle handle for spindle queries

# Footprint sanity: the wheel bottoms must rest on (not through) the terrain.
spindle_world = [
    veh_obj.GetSpindlePos(axle, side)
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Terrain === flat rigid patch the HMMWV drives across
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch_mat.SetYoungModulus(TERRAIN_YOUNG)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver === constant-input driver: steering 0.6, throttle 0.5, held all run
driver = veh.ChDriver(veh_obj)
driver.SetSteering(CONST_STEERING)
driver.SetThrottle(CONST_THROTTLE)
driver.SetBraking(CONST_BRAKING)
driver.Initialize()

# === Sensors (IMU = accelerometer + gyroscope, plus GPS) ===
# All ride on the chassis at the (0, 0, 1) m offset pose; manager.Update() per step.
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(100, 100, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0
)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

imu_pose = chrono.ChFramed(IMU_OFFSET, chrono.QUNIT)
gps_pose = chrono.ChFramed(GPS_OFFSET, chrono.QUNIT)
noise_none = sens.ChNoiseNone()               # noiseless model (prompt specifies none)

accelerometer = sens.ChAccelerometerSensor(chassis, IMU_UPDATE_RATE, imu_pose, noise_none)
accelerometer.SetName("imu_accelerometer")
accelerometer.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(accelerometer)

gyroscope = sens.ChGyroscopeSensor(chassis, IMU_UPDATE_RATE, imu_pose, noise_none)
gyroscope.SetName("imu_gyroscope")
gyroscope.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyroscope)

gps = sens.ChGPSSensor(chassis, GPS_UPDATE_RATE, gps_pose, GPS_REFERENCE, noise_none)
gps.SetName("gps_receiver")
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV GPS + IMU")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # follow the chassis
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-10, -10, 5), chrono.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()
    vis.AddGrid(2.0, 2.0, 50, 50,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
    vis.AttachVehicle(veh_obj)

# === Main loop === render-cadence outer loop; physics + sensors in the inner batch
os.makedirs("frames", exist_ok=True)         # guard against missing output dir
os.makedirs("cam", exist_ok=True)            # motion log + review frames live here

data_f = None
motion_f = None
try:
    # Open CSVs with context managers so they always flush/close, even on error.
    with open("simulation_data.csv", "w", newline="") as data_f, \
         open("cam/motion_log.csv", "w", newline="") as motion_f:
        data_w = csv.writer(data_f)
        data_w.writerow([
            "time",
            "acc_x", "acc_y", "acc_z",         # IMU accelerometer (m/s^2)
            "gyro_x", "gyro_y", "gyro_z",      # IMU gyroscope (rad/s)
            "gps_lat", "gps_lon", "gps_alt",   # GPS fix (deg, deg, m)
        ])
        motion_w = csv.writer(motion_f)
        motion_w.writerow([
            "time", "x", "y", "z", "vx", "vy", "vz", "speed", "yaw",
        ])

        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index
                frame += 1

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                # --- standard vehicle synchronize order ---
                driver.Synchronize(time)
                terrain.Synchronize(time)
                hmmwv.Synchronize(time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                manager.Update()               # pump sensors every physics step

                # --- log IMU + GPS readings (guard each buffer with HasData) ---
                acc = [float("nan")] * 3
                acc_buf = accelerometer.GetMostRecentAccelBuffer()
                if acc_buf.HasData():            # guard: empty before first tick
                    acc = list(acc_buf.GetAccelData())   # numpy array, no index arg

                gyro = [float("nan")] * 3
                gyro_buf = gyroscope.GetMostRecentGyroBuffer()
                if gyro_buf.HasData():
                    gyro = list(gyro_buf.GetGyroData())

                gps_lat = gps_lon = gps_alt = float("nan")
                gps_buf = gps.GetMostRecentGPSBuffer()
                if gps_buf.HasData():
                    g = gps_buf.GetGPSData()     # [lon, lat, alt, time]
                    gps_lon, gps_lat, gps_alt = float(g[0]), float(g[1]), float(g[2])

                data_w.writerow([
                    f"{time:.5f}",
                    acc[0], acc[1], acc[2],
                    gyro[0], gyro[1], gyro[2],
                    gps_lat, gps_lon, gps_alt,
                ])

                # --- log chassis motion ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
                yaw = chassis.GetRot().GetCardanAnglesZYX().z
                motion_w.writerow([
                    f"{time:.5f}", pos.x, pos.y, pos.z,
                    vel.x, vel.y, vel.z, speed, yaw,
                ])

                # --- advance the full subsystem stack (no extra DoStepDynamics) ---
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)        # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= RUN_END:
                    break
except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:                # disk / permission while writing CSV
    import traceback
    traceback.print_exc()
    raise
finally:
    # Context managers above already flushed/closed the writers; nothing left open.
    pass

# === Post-processing === GPS trajectory plot + IMU/motion timeseries from the CSV
gps_lat_series, gps_lon_series = [], []
times, acc_x, gyro_z, speeds = [], [], [], []
with open("simulation_data.csv", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            lat = float(row["gps_lat"])
            lon = float(row["gps_lon"])
        except ValueError:
            lat = lon = float("nan")
        if not (math.isnan(lat) or math.isnan(lon)):
            gps_lat_series.append(lat)
            gps_lon_series.append(lon)
        times.append(float(row["time"]))
        acc_x.append(float(row["acc_x"]))
        gyro_z.append(float(row["gyro_z"]))

with open("cam/motion_log.csv", newline="") as f:
    for row in csv.DictReader(f):
        speeds.append(float(row["speed"]))

# Matplotlib plot of the GPS data: latitude vs longitude trajectory.
fig, axes = plt.subplots(2, 2, figsize=(12, 9))

axes[0, 0].plot(gps_lon_series, gps_lat_series, "-b", marker=".", markersize=2)
axes[0, 0].set_xlabel("Longitude (deg)")
axes[0, 0].set_ylabel("Latitude (deg)")
axes[0, 0].set_title("GPS trajectory (lat vs lon)")
axes[0, 0].ticklabel_format(useOffset=False, style="plain")
axes[0, 0].grid(True)

axes[0, 1].plot(times, speeds[: len(times)] if len(speeds) >= len(times) else speeds)
axes[0, 1].set_xlabel("time (s)")
axes[0, 1].set_ylabel("chassis speed (m/s)")
axes[0, 1].set_title("Vehicle speed")
axes[0, 1].grid(True)

axes[1, 0].plot(times, acc_x)
axes[1, 0].set_xlabel("time (s)")
axes[1, 0].set_ylabel("acc_x (m/s^2)")
axes[1, 0].set_title("IMU accelerometer X")
axes[1, 0].grid(True)

axes[1, 1].plot(times, gyro_z)
axes[1, 1].set_xlabel("time (s)")
axes[1, 1].set_ylabel("gyro_z (rad/s)")
axes[1, 1].set_title("IMU gyroscope yaw rate")
axes[1, 1].grid(True)

fig.tight_layout()
fig.savefig("simulation_timeseries.png", dpi=110)
plt.close(fig)

print(f"Done: {len(times)} samples, {len(gps_lat_series)} GPS fixes logged.")
