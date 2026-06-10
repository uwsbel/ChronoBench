"""GPS + IMU sensors on a wheeled vehicle (PyChrono 9.0.1, Irrlicht).

Model
-----
A full HMMWV (`veh.HMMWV_Full`, SMC contact, AWD, TMEASY tires) drives over a flat
rigid terrain patch. The chassis carries an inertial measurement unit (an
accelerometer + a gyroscope) and a GPS receiver, all managed by a single
`sens.ChSensorManager`. A scripted driver applies a fixed throttle/steering profile
and engages the brakes after a configurable time so the sensor traces show a clear
acceleration, a steady cruise, a turn, and a braking decay.

System type
-----------
SMC (`chrono.ChContactMethod_SMC`) — owned internally by the HMMWV wrapper.

Main bodies
-----------
- HMMWV chassis (sensor mount) + four spindles/wheels (created by the wrapper).
- RigidTerrain flat patch (the support the wheels ride on).

Expected behavior
-----------------
The vehicle accelerates forward, holds a gentle right turn, then brakes to a stop.
The accelerometer logs longitudinal accel spikes at launch and braking; the
gyroscope logs a non-zero yaw rate during the turn; the GPS receiver logs a
latitude/longitude track that drifts away from the reference origin. All channels
are written to CSV and plotted.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / schedule) ===
STEP_SIZE = 2.0e-3                     # integration step (s)
TIRE_STEP_SIZE = 1.0e-3                # TMEASY tire substep (s)
SIM_END = 10.0                         # total simulated time (s)
RENDER_FPS = 30.0                      # review-video frame rate
SENSOR_UPDATE_RATE = 100.0            # IMU / GPS update rate (Hz)

TERRAIN_LENGTH = 200.0                 # rigid patch X extent (m)
TERRAIN_WIDTH = 200.0                  # rigid patch Y extent (m)
TERRAIN_HEIGHT = 0.0                   # top of the patch (m, Z-up)

SUSPENSION_REF_HEIGHT = 0.5            # HMMWV chassis-origin height above wheel-bottom (m)
TIRE_RADIUS = 0.46                     # HMMWV tire radius (m), used in footprint assert
ZTOL = 0.10                            # allowed wheel-bottom clearance vs support top (m)

INIT_X = -90.0                         # spawn X so the vehicle has room to drive (m)
INIT_Y = 0.0                           # spawn Y (m)
INIT_Z = TERRAIN_HEIGHT + SUSPENSION_REF_HEIGHT   # derived chassis-origin Z (m)

# GPS reference datum (longitude, latitude, altitude) — the local origin maps here.
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)

# Scripted driver schedule (s) — accelerate, cruise + turn, then brake.
THROTTLE_RAMP_END = 1.0                # ramp throttle in over the first second
CRUISE_THROTTLE = 0.7                  # steady throttle while cruising
TURN_STEERING = 0.35                   # right-turn steering during cruise
BRAKE_TIME = 6.0                       # engage brakes after this time (s)

LOG_STEP_SIZE = 1.0 / SENSOR_UPDATE_RATE   # how often to sample/log sensor data (s)

# === Derived constants (precomputed once, never recomputed in the loop) ===
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))   # physics steps per frame
log_steps = max(1, round(LOG_STEP_SIZE / STEP_SIZE))           # physics steps per log sample
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless validation run
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check when validating

DATA_CSV = "simulation_data.csv"
MOTION_CSV = os.path.join("cam", "motion_log.csv")
PLOT_PNG = "simulation_timeseries.png"


def scripted_inputs(t):
    """Return (steering, throttle, braking) for the scripted schedule at time t."""
    if t >= BRAKE_TIME:
        return 0.0, 0.0, 1.0                      # full brake, no throttle/steer
    if t < THROTTLE_RAMP_END:
        return 0.0, CRUISE_THROTTLE * (t / THROTTLE_RAMP_END), 0.0   # ramp up straight
    return TURN_STEERING, CRUISE_THROTTLE, 0.0    # cruise with a gentle right turn


def main():
    # === Vehicle (HMMWV_Full wrapper owns its SMC system + chassis/wheels/joints) ===
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire so the car actually drives
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    system = hmmwv.GetSystem()                    # ChSystemSMC owned by the wrapper
    chassis = hmmwv.GetChassisBody()              # cache: main chassis rigid body, reused every step
    vehicle = hmmwv.GetVehicle()                  # cache: ChWheeledVehicle handle, reused every step
    # spindles/wheels: vehicle.GetSpindlePos(axle, side); joints: suspension+steering links inside wrapper

    # === Footprint check (wheels rest on, not through, the rigid patch) ===
    spindle_world = [
        vehicle.GetSpindlePos(axle, side)
        for axle in range(vehicle.GetNumberAxles())
        for side in (veh.LEFT, veh.RIGHT)
    ]
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_HEIGHT - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} vs "
        f"terrain top z={TERRAIN_HEIGHT:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_HEIGHT - wheel_bottom_z:.3f} m"
    )

    # === Terrain (flat rigid support patch under the vehicle) ===
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # === Sensors (IMU = accelerometer + gyroscope; GPS) on the chassis ===
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(
        chrono.ChVector3f(100, 100, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0
    )
    no_noise = sens.ChNoiseNone()                 # ideal (noise-free) sensor models
    chassis_body = chassis                         # the body all three sensors ride on
    sensor_frame = chrono.ChFramed(chrono.VNULL, chrono.QUNIT)   # mounted at chassis origin

    accel = sens.ChAccelerometerSensor(chassis_body, SENSOR_UPDATE_RATE, sensor_frame, no_noise)
    accel.SetName("imu_accelerometer")
    accel.PushFilter(sens.ChFilterAccelAccess())  # expose accel buffer for logging
    manager.AddSensor(accel)

    gyro = sens.ChGyroscopeSensor(chassis_body, SENSOR_UPDATE_RATE, sensor_frame, no_noise)
    gyro.SetName("imu_gyroscope")
    gyro.PushFilter(sens.ChFilterGyroAccess())    # expose gyro (angular rate) buffer
    manager.AddSensor(gyro)

    gps = sens.ChGPSSensor(
        chassis_body, SENSOR_UPDATE_RATE, sensor_frame, GPS_REFERENCE, no_noise
    )
    gps.SetName("gps_receiver")
    gps.PushFilter(sens.ChFilterGPSAccess())      # expose GPS (lat/long/alt) buffer
    manager.AddSensor(gps)

    # === Driver (scripted open-loop schedule via DriverInputs each step) ===
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.0
    driver_inputs.m_steering = 0.0
    driver_inputs.m_braking = 0.0

    # === Visualization === full vehicle Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV with GPS + IMU sensors")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.5)   # chase view behind chassis
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
        vis.AttachVehicle(vehicle)

    # === Output set-up (dirs guarded; CSV opened with context managers) ===
    os.makedirs("frames", exist_ok=True)   # guard against missing frame output dir
    os.makedirs("cam", exist_ok=True)      # guard against missing cam output dir

    gps_data = []                          # list of logged [lat, long, alt, time] samples
    data_file = None
    motion_file = None
    try:
        data_file = open(DATA_CSV, "w", newline="")          # IMU + GPS time series
        motion_file = open(MOTION_CSV, "w", newline="")       # chassis pose/velocity contract
    except (OSError, IOError) as exc:                         # disk full / permission denied
        print(f"[error] could not open output CSV: {exc}")
        raise

    try:
        data_writer = csv.writer(data_file)
        data_writer.writerow([
            "time", "throttle", "steering", "braking",
            "acc_x", "acc_y", "acc_z",
            "gyro_roll", "gyro_pitch", "gyro_yaw",
            "gps_lat", "gps_long", "gps_alt", "gps_time",
        ])
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow([
            "time", "body", "x", "y", "z", "vx", "vy", "vz", "speed",
        ])

        # === Main loop (render-cadence outer loop; physics + sensors in inner batch) ===
        frame = 0
        step = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(render_every):
                time = system.GetChTime()
                steering, throttle, braking = scripted_inputs(time)
                driver_inputs.m_steering = steering
                driver_inputs.m_throttle = throttle
                driver_inputs.m_braking = braking

                # Subsystem synchronize (no separate driver object — inputs set above).
                terrain.Synchronize(time)
                hmmwv.Synchronize(time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                manager.Update()   # pump sensors every physics step (post-step pose)

                # Log sensor + motion data at the configured cadence.
                if step % log_steps == 0:
                    acc_buf = accel.GetMostRecentAccelBuffer()   # may be empty before first tick
                    gyro_buf = gyro.GetMostRecentGyroBuffer()
                    gps_buf = gps.GetMostRecentGPSBuffer()

                    ax = ay = az = float("nan")
                    if acc_buf.HasData():            # guard: sensor not yet filled
                        a = acc_buf.GetAccelData()   # numpy [X, Y, Z], no index
                        ax, ay, az = float(a[0]), float(a[1]), float(a[2])

                    groll = gpitch = gyaw = float("nan")
                    if gyro_buf.HasData():           # guard: sensor not yet filled
                        g = gyro_buf.GetGyroData()   # numpy [Roll, Pitch, Yaw], no index
                        groll, gpitch, gyaw = float(g[0]), float(g[1]), float(g[2])

                    glat = glong = galt = gtime = float("nan")
                    if gps_buf.HasData():            # guard: sensor not yet filled
                        gd = gps_buf.GetGPSData()    # numpy [Lat, Long, Alt, Time], no index
                        glat, glong, galt, gtime = (
                            float(gd[0]), float(gd[1]), float(gd[2]), float(gd[3])
                        )
                        gps_data.append([glat, glong, galt, gtime])

                    data_writer.writerow([
                        f"{time:.5f}", f"{throttle:.4f}", f"{steering:.4f}", f"{braking:.4f}",
                        f"{ax:.5f}", f"{ay:.5f}", f"{az:.5f}",
                        f"{groll:.6f}", f"{gpitch:.6f}", f"{gyaw:.6f}",
                        f"{glat:.8f}", f"{glong:.8f}", f"{galt:.4f}", f"{gtime:.5f}",
                    ])

                    pos = chassis.GetPos()
                    vel = chassis.GetPosDt()
                    speed = vel.Length()
                    motion_writer.writerow([
                        f"{time:.5f}", "chassis",
                        f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                        f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}", f"{speed:.5f}",
                    ])

                # Advance the full subsystem stack (wrapper Advance steps the system).
                terrain.Advance(STEP_SIZE)
                hmmwv.Advance(STEP_SIZE)     # advances the wrapper-owned ChSystem
                if not HEADLESS:
                    vis.Advance(STEP_SIZE)
                step += 1

                if system.GetChTime() >= run_end:
                    break

        print("GPS Data: ", gps_data)   # report the logged GPS track

    except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close writers even if a step diverges mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing (plot the logged IMU + GPS channels vs time) ===
    try:
        rows = []
        with open(DATA_CSV, "r", newline="") as f:   # re-read what we just wrote
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
    except (OSError, IOError) as exc:                 # missing/locked CSV
        print(f"[warn] could not read CSV for plotting: {exc}")
        rows = []

    if rows:
        t = np.array([float(r["time"]) for r in rows])
        acc_x = np.array([float(r["acc_x"]) for r in rows])
        acc_z = np.array([float(r["acc_z"]) for r in rows])
        gyro_yaw = np.array([float(r["gyro_yaw"]) for r in rows])
        gps_lat = np.array([float(r["gps_lat"]) for r in rows])
        gps_long = np.array([float(r["gps_long"]) for r in rows])

        fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=False)
        axes[0].plot(t, acc_x, label="acc_x")
        axes[0].plot(t, acc_z, label="acc_z")
        axes[0].set_ylabel("accel (m/s^2)")
        axes[0].set_xlabel("time (s)")
        axes[0].legend(); axes[0].grid(True)
        axes[0].set_title("Accelerometer")

        axes[1].plot(t, gyro_yaw, color="tab:green", label="gyro_yaw")
        axes[1].set_ylabel("yaw rate (rad/s)")
        axes[1].set_xlabel("time (s)")
        axes[1].legend(); axes[1].grid(True)
        axes[1].set_title("Gyroscope (yaw)")

        axes[2].plot(gps_long, gps_lat, color="tab:red")
        axes[2].set_xlabel("longitude (deg)")
        axes[2].set_ylabel("latitude (deg)")
        axes[2].grid(True)
        axes[2].set_title("GPS track")

        fig.tight_layout()
        fig.savefig(PLOT_PNG, dpi=110)
        plt.close(fig)
        print(f"Wrote {PLOT_PNG} ({len(rows)} samples)")


if __name__ == "__main__":
    main()
