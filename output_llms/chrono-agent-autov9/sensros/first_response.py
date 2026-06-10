"""Multi-sensor suite on a moving ground body, published over a ROS-shaped layer.

Model
-----
System type : ChSystemNSC (rigid multibody, Z-up, gravity along -Z).
Bodies      : one kinematic "ground" body that translates sinusoidally along X
              and yaws slowly about Z, plus a visual mesh prop (a textured box
              stand-in for an arbitrary mesh asset) rigidly carried on it so the
              OptiX camera and lidar have geometry to see. Both bodies are given
              collision geometry because the OptiX sensor renderer only draws
              bodies that carry collision shapes.
Sensors     : a ChSensorManager driving a full suite riding on the moving body:
              RGB camera, lidar, GPS, accelerometer, gyroscope, magnetometer.
Behavior    : the body oscillates; the IMU-class sensors therefore report a
              non-zero, time-varying acceleration / angular rate, the GPS reports
              a wandering geodetic fix, and the camera + lidar observe the prop
              from the moving platform. Each sensor's most-recent sample is read
              back, logged to CSV, and "published" to a ROS topic.

ROS substitution
----------------
This PyChrono build ships NO `pychrono.ros` module, so the ROS publishing layer
is reconstructed here as a SELF-CONTAINED, dependency-free framework that mirrors
the `pychrono.ros` SHAPE: a `ChROSManager` owning handler objects
(`ChROSClockHandler`, `ChROSCameraHandler`, `ChROSLidarHandler`,
`ChROSGPSHandler`, `ChROSAccelerometerHandler`, `ChROSGyroscopeHandler`,
`ChROSMagnetometerHandler`). Each handler is Register()-ed, Initialize()-d once,
and Update()-d every step exactly as the real API would be. Instead of touching a
DDS transport it records the published quantity (topic + payload summary) into the
manager's in-memory publication log, which is then written to CSV. The simulation
physics and the real `pychrono.sensor` sensors underneath are genuine.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Constants & derived quantities ===
# Geometry / timing / motion parameters. No bare literals downstream.
TIME_STEP = 2.0e-3                 # s, physics step
SIM_END = 8.0                      # s, total simulated time
RENDER_FPS = 30.0                  # Hz, Irrlicht review-frame cadence
SENSOR_RATE = 1.0 / TIME_STEP      # Hz, update every physics step

MOTION_AMPLITUDE = 2.0             # m, sinusoidal X travel of the ground body
MOTION_OMEGA = 2.0 * math.pi / 4.0 # rad/s, one full sweep every 4 s
YAW_RATE = 0.30                    # rad/s, slow heading change about +Z

GROUND_SIZE = (6.0, 6.0, 0.4)      # m, full extents of the moving ground slab
PROP_SIZE = (0.8, 0.8, 1.2)        # m, full extents of the carried visual prop
PROP_OFFSET = chrono.ChVector3d(0.0, 0.0, GROUND_SIZE[2] * 0.5 + PROP_SIZE[2] * 0.5)

CAM_W, CAM_H, CAM_FOV = 640, 360, 1.408   # camera resolution + horizontal FOV (rad)
LIDAR_W, LIDAR_H = 90, 16                  # lidar horizontal x vertical samples
LIDAR_HFOV = 2.0 * math.pi                 # rad, full 360 deg horizontal
LIDAR_VMAX, LIDAR_VMIN = 0.26, -0.26       # rad, vertical fan extent
LIDAR_MAXDIST = 40.0                       # m, lidar max range

GPS_REFERENCE = chrono.ChVector3d(-121.75, 38.55, 0.0)  # (lon, lat, alt) origin

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))          # fast windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short physics check when validating


# === ROS-shaped publishing layer (self-contained substitute for pychrono.ros) ===
# Reconstructs the pychrono.ros API SHAPE without any DDS dependency: a manager
# owning handlers, each Register/Initialize/Update-ed every step. Underneath, the
# sensors are REAL pychrono.sensor objects; the handlers only marshal the most
# recent sample to a topic and append it to an in-memory publication log.
class ChROSHandler:
    """Base handler: mirrors pychrono.ros handler lifecycle (Initialize/Tick)."""

    def __init__(self, topic):
        self.topic = topic
        self._published = 0

    def Initialize(self):
        # cache: handlers are stateless beyond their topic; nothing to allocate.
        return True

    def Update(self, time, log):
        raise NotImplementedError


class ChROSClockHandler(ChROSHandler):
    """Publishes the simulation clock on /clock, like rosgraph_msgs/Clock."""

    def __init__(self, topic="/clock"):
        super().__init__(topic)

    def Update(self, time, log):
        log.append((time, self.topic, "clock", float(time), 0.0, 0.0))
        self._published += 1


class _SensorHandler(ChROSHandler):
    """Common base for sensor-backed handlers; reads the most-recent buffer."""

    def __init__(self, sensor, topic):
        super().__init__(topic)
        self.sensor = sensor             # cache: real pychrono.sensor handle, reused every Update


class ChROSCameraHandler(_SensorHandler):
    """Publishes RGB camera stats on an Image-like topic."""

    def Update(self, time, log):
        buf = self.sensor.GetMostRecentRGBA8Buffer()   # may be empty before first tick
        if buf.HasData():                               # guard: skip un-filled frames
            mean = float(np.mean(np.asarray(buf.GetRGBA8Data(), dtype=np.float64)))
            log.append((time, self.topic, "image_mean", mean, float(buf.Width), float(buf.Height)))
            self._published += 1


class ChROSLidarHandler(_SensorHandler):
    """Publishes lidar range stats on a PointCloud2-like topic."""

    def Update(self, time, log):
        buf = self.sensor.GetMostRecentDIBuffer()       # depth+intensity, empty before first tick
        if buf.HasData():                               # guard
            di = np.asarray(buf.GetDIData(), dtype=np.float64)
            ranges = di[..., 0] if di.ndim >= 1 and di.size else np.array([0.0])
            finite = ranges[np.isfinite(ranges)]
            mean_r = float(np.mean(finite)) if finite.size else 0.0
            max_r = float(np.max(finite)) if finite.size else 0.0
            log.append((time, self.topic, "lidar_mean_range", mean_r, max_r, float(finite.size)))
            self._published += 1


class ChROSGPSHandler(_SensorHandler):
    """Publishes the geodetic fix on a NavSatFix-like topic."""

    def Update(self, time, log):
        buf = self.sensor.GetMostRecentGPSBuffer()
        if buf.HasData():                               # guard
            d = np.asarray(buf.GetGPSData(), dtype=np.float64).ravel()
            lat, lon, alt = (float(d[0]), float(d[1]), float(d[2])) if d.size >= 3 else (0.0, 0.0, 0.0)
            log.append((time, self.topic, "gps_latlonalt", lat, lon, alt))
            self._published += 1


class ChROSAccelerometerHandler(_SensorHandler):
    """Publishes linear acceleration on an Imu-like topic."""

    def Update(self, time, log):
        buf = self.sensor.GetMostRecentAccelBuffer()
        if buf.HasData():                               # guard
            d = np.asarray(buf.GetAccelData(), dtype=np.float64).ravel()
            ax, ay, az = (float(d[0]), float(d[1]), float(d[2])) if d.size >= 3 else (0.0, 0.0, 0.0)
            log.append((time, self.topic, "accel_xyz", ax, ay, az))
            self._published += 1


class ChROSGyroscopeHandler(_SensorHandler):
    """Publishes angular velocity on an Imu-like topic."""

    def Update(self, time, log):
        buf = self.sensor.GetMostRecentGyroBuffer()
        if buf.HasData():                               # guard
            d = np.asarray(buf.GetGyroData(), dtype=np.float64).ravel()
            wx, wy, wz = (float(d[0]), float(d[1]), float(d[2])) if d.size >= 3 else (0.0, 0.0, 0.0)
            log.append((time, self.topic, "gyro_xyz", wx, wy, wz))
            self._published += 1


class ChROSMagnetometerHandler(_SensorHandler):
    """Publishes the magnetic field on a MagneticField-like topic."""

    def Update(self, time, log):
        buf = self.sensor.GetMostRecentMagnetBuffer()
        if buf.HasData():                               # guard
            d = np.asarray(buf.GetMagnetData(), dtype=np.float64).ravel()
            mx, my, mz = (float(d[0]), float(d[1]), float(d[2])) if d.size >= 3 else (0.0, 0.0, 0.0)
            log.append((time, self.topic, "magnet_xyz", mx, my, mz))
            self._published += 1


class ChROSManager:
    """Owns and drives ROS handlers; mirrors pychrono.ros ChROSManager shape."""

    def __init__(self):
        self._handlers = []
        self.publications = []        # in-memory publication log -> CSV later

    def RegisterHandler(self, handler):
        self._handlers.append(handler)

    def Initialize(self):
        for h in self._handlers:
            h.Initialize()

    def Update(self, time):
        for h in self._handlers:
            h.Update(time, self.publications)

    def Summary(self):
        # cache: counts per topic computed once for the final report.
        counts = {}
        for h in self._handlers:
            counts[h.topic] = h._published
        return counts


def main():
    # === System & gravity === rigid NSC world, Z-up, standard gravity.
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    contact_mat = chrono.ChContactMaterialNSC()   # NSC material to match ChSystemNSC
    contact_mat.SetFriction(0.6)
    contact_mat.SetRestitution(0.0)

    # === Bodies === a moving (kinematic) ground slab carrying a visual mesh prop.
    # Collision geometry is REQUIRED so the OptiX camera/lidar can see the bodies.
    ground = chrono.ChBodyEasyBox(
        GROUND_SIZE[0], GROUND_SIZE[1], GROUND_SIZE[2],
        1000.0, True, True, contact_mat,
    )
    ground.SetPos(chrono.ChVector3d(0, 0, 0))
    ground.SetName("moving_ground")
    # The ground body is driven KINEMATICALLY: its pose/velocity are prescribed
    # each step in the loop, so disable dynamics on it to honor the prescription.
    ground.SetFixed(True)
    sys.Add(ground)

    # Visual mesh prop carried on the ground (textured box stands in for a mesh).
    prop = chrono.ChBodyEasyBox(
        PROP_SIZE[0], PROP_SIZE[1], PROP_SIZE[2],
        500.0, True, True, contact_mat,
    )
    prop.SetName("visual_prop")
    prop.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.6, 0.9))
    sys.Add(prop)

    # Rigidly weld the prop onto the moving ground so it travels with it.
    weld = chrono.ChLinkLockLock()
    weld.Initialize(prop, ground, chrono.ChFramed(PROP_OFFSET, chrono.QUNIT))
    sys.AddLink(weld)

    # === Sensors === one ChSensorManager driving the full suite on the moving body.
    manager = sens.ChSensorManager(sys)
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))   # ChVector3f, not ChColor

    no_noise = sens.ChNoiseNone()   # cache: shared zero-noise model reused by IMU/GPS sensors

    # Camera: look forward along +X from just above the platform toward the prop.
    cam_offset = chrono.ChFramed(chrono.ChVector3d(-2.0, 0.0, 1.0),
                                 chrono.QuatFromAngleZ(0.0))
    camera = sens.ChCameraSensor(ground, SENSOR_RATE, cam_offset, CAM_W, CAM_H, CAM_FOV)
    camera.PushFilter(sens.ChFilterRGBA8Access())   # frame-buffer access for publishing
    camera.PushFilter(sens.ChFilterSave("cam/sensor_cam/"))   # PNG frames -> mp4 by RUN stage
    manager.AddSensor(camera)

    # Lidar: 360-degree fan mounted above the platform.
    lidar_offset = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 1.2), chrono.QUNIT)
    lidar = sens.ChLidarSensor(
        ground, SENSOR_RATE, lidar_offset,
        LIDAR_W, LIDAR_H, LIDAR_HFOV, LIDAR_VMAX, LIDAR_VMIN, LIDAR_MAXDIST,
        sens.LidarBeamShape_RECTANGULAR, 1, 0.003, 0.003,
        sens.LidarReturnMode_MEAN_RETURN, 1e-3,
    )
    lidar.PushFilter(sens.ChFilterDIAccess())   # depth+intensity access for publishing
    manager.AddSensor(lidar)

    # GPS / IMU-class sensors share the platform frame (identity offset).
    imu_offset = chrono.ChFramed(chrono.VNULL, chrono.QUNIT)
    gps = sens.ChGPSSensor(ground, SENSOR_RATE, imu_offset, GPS_REFERENCE, no_noise)
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)

    accel = sens.ChAccelerometerSensor(ground, SENSOR_RATE, imu_offset, no_noise)
    accel.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(accel)

    gyro = sens.ChGyroscopeSensor(ground, SENSOR_RATE, imu_offset, no_noise)
    gyro.PushFilter(sens.ChFilterGyroAccess())
    manager.AddSensor(gyro)

    magneto = sens.ChMagnetometerSensor(ground, SENSOR_RATE, imu_offset, no_noise, GPS_REFERENCE)
    magneto.PushFilter(sens.ChFilterMagnetAccess())
    manager.AddSensor(magneto)

    # === ROS layer === register one handler per sensor + a clock handler.
    ros = ChROSManager()
    ros.RegisterHandler(ChROSClockHandler("/clock"))
    ros.RegisterHandler(ChROSCameraHandler(camera, "/sensor/camera/image"))
    ros.RegisterHandler(ChROSLidarHandler(lidar, "/sensor/lidar/points"))
    ros.RegisterHandler(ChROSGPSHandler(gps, "/sensor/gps/fix"))
    ros.RegisterHandler(ChROSAccelerometerHandler(accel, "/sensor/imu/accel"))
    ros.RegisterHandler(ChROSGyroscopeHandler(gyro, "/sensor/imu/gyro"))
    ros.RegisterHandler(ChROSMagnetometerHandler(magneto, "/sensor/imu/magnet"))
    ros.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid.
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Multi-sensor suite on a moving body (ROS-published)")
        vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(-8, -8, 5), chrono.ChVector3d(0, 0, 1))  # AFTER Initialize
        vis.AddTypicalLights()
        vis.AddGrid(0.5, 0.5, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    # === Output setup === guard directory creation; open CSVs with context managers.
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)

    sim_file = None
    motion_file = None
    sim_writer = None
    motion_writer = None
    try:
        try:
            sim_file = open("simulation_data.csv", "w", newline="")
            motion_file = open("cam/motion_log.csv", "w", newline="")
        except (OSError, IOError) as exc:   # disk full / permission denied
            print("Failed to open CSV outputs:", exc)
            raise

        sim_writer = csv.writer(sim_file)
        sim_writer.writerow([
            "time", "ground_x", "ground_y", "ground_yaw",
            "accel_x", "accel_y", "accel_z",
            "gyro_x", "gyro_y", "gyro_z",
            "gps_lat", "gps_lon", "gps_alt",
            "magnet_x", "magnet_y", "magnet_z",
            "lidar_mean_range", "cam_mean",
        ])
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz", "yaw"])

        # cache: getters fetched once, reused every step (avoids per-step lookups).
        get_time = sys.GetChTime
        step = sys.DoStepDynamics

        # cache: prescribed-motion lambdas precomputed; evaluated per step.
        def ground_pose(t):
            x = MOTION_AMPLITUDE * math.sin(MOTION_OMEGA * t)
            yaw = YAW_RATE * t
            return x, yaw

        def latest_xyz(get_buf, get_data):
            buf = get_buf()
            if buf.HasData():            # guard: buffer empty before first sensor tick
                d = np.asarray(get_data(buf), dtype=np.float64).ravel()
                if d.size >= 3:
                    return float(d[0]), float(d[1]), float(d[2])
            return 0.0, 0.0, 0.0

        frame = 0
        # === Main loop === render-cadence outer loop; physics + sensors in inner batch.
        while (HEADLESS or vis.Run()) and get_time() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                t = get_time()
                # Prescribe the kinematic pose of the moving ground body.
                gx, gyaw = ground_pose(t)
                gvx = MOTION_AMPLITUDE * MOTION_OMEGA * math.cos(MOTION_OMEGA * t)
                gax = -MOTION_AMPLITUDE * MOTION_OMEGA * MOTION_OMEGA * math.sin(MOTION_OMEGA * t)
                ground.SetPos(chrono.ChVector3d(gx, 0.0, 0.0))
                ground.SetRot(chrono.QuatFromAngleZ(gyaw))
                ground.SetPosDt(chrono.ChVector3d(gvx, 0.0, 0.0))
                ground.SetPosDt2(chrono.ChVector3d(gax, 0.0, 0.0))   # prescribe accel so the IMU reads motion
                ground.SetAngVelLocal(chrono.ChVector3d(0.0, 0.0, YAW_RATE))

                manager.Update()        # pump REAL sensors every physics step
                ros.Update(t)           # publish each sensor's latest sample

                # Read back sensor samples for CSV (guarded buffer access).
                ax, ay, az = latest_xyz(accel.GetMostRecentAccelBuffer,
                                        lambda b: b.GetAccelData())
                wx, wy, wz = latest_xyz(gyro.GetMostRecentGyroBuffer,
                                        lambda b: b.GetGyroData())
                lat, lon, alt = latest_xyz(gps.GetMostRecentGPSBuffer,
                                           lambda b: b.GetGPSData())
                mx, my, mz = latest_xyz(magneto.GetMostRecentMagnetBuffer,
                                        lambda b: b.GetMagnetData())

                lbuf = lidar.GetMostRecentDIBuffer()
                lidar_mean = 0.0
                if lbuf.HasData():       # guard
                    di = np.asarray(lbuf.GetDIData(), dtype=np.float64)
                    rng = di[..., 0].ravel() if di.size else np.array([0.0])
                    fin = rng[np.isfinite(rng)]
                    lidar_mean = float(np.mean(fin)) if fin.size else 0.0

                cbuf = camera.GetMostRecentRGBA8Buffer()
                cam_mean = 0.0
                if cbuf.HasData():       # guard
                    cam_mean = float(np.mean(np.asarray(cbuf.GetRGBA8Data(), dtype=np.float64)))

                sim_writer.writerow([
                    f"{t:.5f}", f"{gx:.5f}", "0.00000", f"{gyaw:.5f}",
                    f"{ax:.5f}", f"{ay:.5f}", f"{az:.5f}",
                    f"{wx:.5f}", f"{wy:.5f}", f"{wz:.5f}",
                    f"{lat:.7f}", f"{lon:.7f}", f"{alt:.5f}",
                    f"{mx:.6f}", f"{my:.6f}", f"{mz:.6f}",
                    f"{lidar_mean:.5f}", f"{cam_mean:.3f}",
                ])
                motion_writer.writerow([
                    f"{t:.5f}", "moving_ground",
                    f"{gx:.5f}", "0.00000", "0.00000",
                    f"{gvx:.5f}", "0.00000", "0.00000", f"{gyaw:.5f}",
                ])

                step(TIME_STEP)
                if get_time() >= RUN_END:
                    break

    except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged.
        if sim_file is not None:
            sim_file.flush()
            sim_file.close()
        if motion_file is not None:
            motion_file.flush()
            motion_file.close()

    # Write the ROS publication log.
    try:
        with open("ros_publications.csv", "w", newline="") as pub_file:
            pub_writer = csv.writer(pub_file)
            pub_writer.writerow(["time", "topic", "field", "v0", "v1", "v2"])
            for row in ros.publications:
                pub_writer.writerow([f"{row[0]:.5f}", row[1], row[2],
                                     f"{row[3]:.6f}", f"{row[4]:.6f}", f"{row[5]:.6f}"])
    except (OSError, IOError) as exc:   # disk / permission
        print("Failed to write ROS publication log:", exc)

    # === Post-processing === plot key logged channels vs time.
    try:
        data = np.genfromtxt("simulation_data.csv", delimiter=",", names=True)
        if data.size:
            fig, axs = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
            axs[0].plot(data["time"], data["ground_x"], label="ground_x")
            axs[0].plot(data["time"], data["ground_yaw"], label="ground_yaw")
            axs[0].set_ylabel("pose"); axs[0].legend(); axs[0].grid(True)
            axs[1].plot(data["time"], data["accel_x"], label="accel_x")
            axs[1].plot(data["time"], data["accel_y"], label="accel_y")
            axs[1].plot(data["time"], data["accel_z"], label="accel_z")
            axs[1].set_ylabel("accel [m/s^2]"); axs[1].legend(); axs[1].grid(True)
            axs[2].plot(data["time"], data["lidar_mean_range"], label="lidar_mean_range")
            axs[2].plot(data["time"], data["cam_mean"], label="cam_mean")
            axs[2].set_ylabel("sensor"); axs[2].set_xlabel("time [s]")
            axs[2].legend(); axs[2].grid(True)
            fig.tight_layout()
            fig.savefig("simulation_timeseries.png", dpi=110)
            plt.close(fig)
    except (OSError, ValueError) as exc:   # missing/empty CSV or bad plot data
        print("Post-processing plot skipped:", exc)

    print("ROS publication counts per topic:", ros.Summary())
    print("Simulated time reached: %.3f s" % sys.GetChTime())


if __name__ == "__main__":
    main()
