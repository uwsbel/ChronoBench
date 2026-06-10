"""HMMWV with onboard IMU + GPS sensors on rigid terrain (PyChrono 9.0.1, Irrlicht).

Model
-----
A full-model four-wheel HMMWV (wrapper-owned ChSystemSMC, SMC contact) drives
forward under a scripted data driver on a flat RigidTerrain patch. Two sensors are
rigidly mounted on the chassis body and managed by a ChSensorManager:
  * an IMU built from a ChAccelerometerSensor + ChGyroscopeSensor (specific force
    and angular rate of the chassis), and
  * a ChGPSSensor reporting latitude / longitude / altitude relative to a fixed
    geodetic reference.

Both sensors are noise-free (ChNoiseNone) and are pumped once per physics step via
manager.Update(); their most-recent buffers are guarded with HasData() before read
and logged to CSV. The total vehicle mass is queried and printed once.

Visualization is a full Irrlicht window (the IMU/GPS produce no image, so the review
video comes from the Irrlicht scene); review frames are written to frames/*.png and
assembled into a video by the run stage.

Expected behavior: the HMMWV accelerates forward, the accelerometer shows a non-zero
forward specific force during the throttle phase plus the ~9.81 m/s^2 gravity term,
the gyroscope stays small (near-straight driving), and the GPS longitude drifts as
the vehicle translates along +X.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants ===  geometry / physics / sensor parameters (no bare literals downstream)
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire substep (s)
SIM_END = 8.0                      # simulated duration (s)
RENDER_FPS = 30.0                  # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: physics steps per frame

TERRAIN_LENGTH = 200.0             # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0              # rigid patch Y extent (m)
TERRAIN_TOP_Z = 0.0                # top surface height (m)

INIT_X = -80.0                     # spawn near one end so the vehicle drives across +X
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5        # HMMWV chassis-origin height above wheel-bottom at rest (m)
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT
TIRE_RADIUS = 0.46                 # approximate HMMWV tire radius (m), for footprint assert
ZTOL = 0.10                        # allowed wheel-bottom clearance vs support top (m)

SENSOR_UPDATE_RATE = 100.0         # IMU/GPS update rate (Hz)
GPS_REFERENCE = chrono.ChVector3d(-43.6, 172.6, 200.0)  # geodetic origin (lat, long, alt)
IMU_OFFSET = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.4), chrono.QUNIT)  # on chassis
GPS_OFFSET = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 1.0), chrono.QUNIT)  # on chassis

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating

os.makedirs("frames", exist_ok=True)  # guard against missing output dir for review frames

# === Vehicle (full HMMWV; wrapper creates + owns its ChSystemSMC) ===
# WHY: the wrapper builds the system, chassis, spindles, suspension + steering joints
# internally; we initialize first, then take handles so the essentials are visible.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)  # grippy tire so the vehicle actually drives
hmmwv.SetTireStepSize(TIRE_STEP)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()               # cache: vehicle subsystem handle, reused every step
# spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering built by wrapper

vehicle_mass = veh_obj.GetMass()           # cache: total vehicle mass, queried once
print(f"HMMWV total vehicle mass = {vehicle_mass:.2f} kg")

# === Footprint assert (wheel bottoms rest on the rigid patch, not through it) ===
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into support: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs support top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Terrain (flat rigid patch under the vehicle) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver (scripted data driver: brief settle, then steady forward throttle) ===
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(1.0, 0.0, 0.6, 0.0, 0.0),
    veh.DataDriverEntry(SIM_END, 0.0, 0.6, 0.0, 0.0),
])
driver = veh.ChDataDriver(veh_obj, driver_data)
driver.Initialize()

# === Sensors (IMU = accelerometer + gyroscope, plus GPS; all on the chassis) ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(10, 10, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

noise_none = sens.ChNoiseNone()  # noise-free model shared by all sensors

acc_sensor = sens.ChAccelerometerSensor(chassis, SENSOR_UPDATE_RATE, IMU_OFFSET, noise_none)
acc_sensor.SetName("IMU_accelerometer")
acc_sensor.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc_sensor)

gyro_sensor = sens.ChGyroscopeSensor(chassis, SENSOR_UPDATE_RATE, IMU_OFFSET, noise_none)
gyro_sensor.SetName("IMU_gyroscope")
gyro_sensor.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro_sensor)

gps_sensor = sens.ChGPSSensor(chassis, SENSOR_UPDATE_RATE, GPS_OFFSET, GPS_REFERENCE, noise_none)
gps_sensor.SetName("GPS")
gps_sensor.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps_sensor)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV with IMU + GPS sensors")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddGrid(2.0, 2.0, 50, 50,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z + 0.01), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))  # ground reference grid
    vis.AttachVehicle(veh_obj)

# === Main loop === render-cadence outer loop; sensors + CSV logged every physics step
data_file = None
motion_file = None
try:
    data_file = open("simulation_data.csv", "w", newline="")        # IMU + GPS readings
    motion_file = open("motion_log.csv", "w", newline="")           # chassis pose/velocity
    data_writer = csv.writer(data_file)
    motion_writer = csv.writer(motion_file)
    data_writer.writerow([
        "time", "acc_x", "acc_y", "acc_z",
        "gyro_roll", "gyro_pitch", "gyro_yaw",
        "gps_lat", "gps_long", "gps_alt",
    ])
    motion_writer.writerow([
        "time", "x", "y", "z", "vx", "vy", "vz", "speed",
    ])

    last_acc = (0.0, 0.0, 0.0)
    last_gyro = (0.0, 0.0, 0.0)
    last_gps = (GPS_REFERENCE.x, GPS_REFERENCE.y, GPS_REFERENCE.z)

    frame = 0
    while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
            frame += 1

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            # synchronize subsystems
            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(time, driver_inputs)

            # pump sensors so they see this post-step pose
            manager.Update()

            # read most-recent sensor buffers (guard: empty until first sensor tick)
            acc_buf = acc_sensor.GetMostRecentAccelBuffer()
            if acc_buf.HasData():
                d = acc_buf.GetAccelData()      # numpy [X, Y, Z] specific force (m/s^2)
                last_acc = (float(d[0]), float(d[1]), float(d[2]))
            gyro_buf = gyro_sensor.GetMostRecentGyroBuffer()
            if gyro_buf.HasData():
                d = gyro_buf.GetGyroData()      # numpy [Roll, Pitch, Yaw] rate (rad/s)
                last_gyro = (float(d[0]), float(d[1]), float(d[2]))
            gps_buf = gps_sensor.GetMostRecentGPSBuffer()
            if gps_buf.HasData():
                d = gps_buf.GetGPSData()        # numpy [Longitude, Latitude, Altitude, Time]
                last_gps = (float(d[1]), float(d[0]), float(d[2]))  # store as (lat, long, alt)

            data_writer.writerow([
                f"{time:.5f}",
                f"{last_acc[0]:.6f}", f"{last_acc[1]:.6f}", f"{last_acc[2]:.6f}",
                f"{last_gyro[0]:.6f}", f"{last_gyro[1]:.6f}", f"{last_gyro[2]:.6f}",
                f"{last_gps[0]:.8f}", f"{last_gps[1]:.8f}", f"{last_gps[2]:.4f}",
            ])

            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            motion_writer.writerow([
                f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}", f"{vel.Length():.5f}",
            ])

            # advance subsystems (hmmwv.Advance steps the wrapper-owned system)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)
            if not HEADLESS:
                vis.Advance(TIME_STEP)

            if system.GetChTime() >= RUN_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:           # disk / permission failure on the CSV writers
    import traceback
    traceback.print_exc()
    raise
finally:
    # flush + close CSV writers even if a step diverged mid-run
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing === plot the logged sensor + motion channels vs time
try:
    arr = np.genfromtxt("simulation_data.csv", delimiter=",", names=True)
    motion = np.genfromtxt("motion_log.csv", delimiter=",", names=True)
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    axes[0, 0].plot(arr["time"], arr["acc_x"], label="acc_x")
    axes[0, 0].plot(arr["time"], arr["acc_y"], label="acc_y")
    axes[0, 0].plot(arr["time"], arr["acc_z"], label="acc_z")
    axes[0, 0].set_title("IMU accelerometer (m/s^2)")
    axes[0, 0].set_xlabel("time (s)"); axes[0, 0].legend()

    axes[0, 1].plot(arr["time"], arr["gyro_roll"], label="roll")
    axes[0, 1].plot(arr["time"], arr["gyro_pitch"], label="pitch")
    axes[0, 1].plot(arr["time"], arr["gyro_yaw"], label="yaw")
    axes[0, 1].set_title("IMU gyroscope (rad/s)")
    axes[0, 1].set_xlabel("time (s)"); axes[0, 1].legend()

    axes[1, 0].plot(arr["gps_long"], arr["gps_lat"])
    axes[1, 0].set_title("GPS track (long vs lat)")
    axes[1, 0].set_xlabel("longitude"); axes[1, 0].set_ylabel("latitude")

    axes[1, 1].plot(motion["time"], motion["speed"], label="speed")
    axes[1, 1].plot(motion["time"], motion["x"], label="x")
    axes[1, 1].set_title("Chassis speed (m/s) & x (m)")
    axes[1, 1].set_xlabel("time (s)"); axes[1, 1].legend()

    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
except (OSError, ValueError) as exc:        # missing/empty CSV or malformed rows
    import traceback
    traceback.print_exc()

print("done")
