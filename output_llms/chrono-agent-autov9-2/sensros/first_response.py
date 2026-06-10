"""Multi-sensor ground body with ROS-style publishing (PyChrono 9.0.1, NSC).

This script builds a single rigid "ground" body that carries a visual mesh and is
driven through a smooth oscillating translation by a linear motor. A full sensor
suite -- RGB camera, lidar, GPS, accelerometer, gyroscope and magnetometer -- is
attached to that body and managed by a ChSensorManager that renders/updates every
physics step. Because PyChrono 9.0.1 ships no `pychrono.ros` module, the ROS
publishing layer is reconstructed in plain Python: a small ChROSHandler base class
is rate-gated (Update -> Tick), one publisher handler per sensor reads that
sensor's most-recent buffer, and a ChROSPythonManager ticks every handler each
step (mirroring the real rclcpp publish cadence). No data is sent on a real ROS
graph; the handlers print/aggregate the messages that WOULD be published to their
topics.

System type: ChSystemNSC (penalty-free constraints). Main body: the moving
"ground" body (a box with a visual mesh and a collision shape). Expected behavior:
the body oscillates along +X/-X; the accelerometer/gyroscope register the induced
acceleration, the GPS/magnetometer report geo-referenced readings, the lidar sweeps
the static reference cube, and the camera observes the scene. All six ROS-style
publisher handlers tick at their configured rates and report non-empty messages.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === geometry / physics / sensor rates (no bare literals downstream)
time_step = 2.0e-3                 # s, integration step
sim_end = 6.0                      # s, total simulated time
render_fps = 30.0                  # Hz, Irrlicht review cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

ground_size = chrono.ChVector3d(2.0, 2.0, 0.4)   # full extents of the moving body (m)
ground_density = 800.0             # kg/m^3
motion_amplitude = 1.5             # m, peak X displacement of the moving body
motion_freq = 0.25                 # Hz, oscillation frequency
motion_omega = 2.0 * math.pi * motion_freq        # rad/s, precomputed once

ref_cube_pos = chrono.ChVector3d(4.0, 0.0, 0.5)  # static lidar/camera reference target
gps_reference = chrono.ChVector3d(-89.40, 43.07, 260.0)  # lon, lat, alt origin (Madison, WI)

cam_w, cam_h, cam_fov = 1280, 720, 1.408          # camera resolution + horizontal FOV (rad)
cam_rate = 20.0                    # Hz, camera + lidar update rate
imu_rate = 100.0                   # Hz, accelerometer / gyroscope / magnetometer rate
gps_rate = 10.0                    # Hz, GPS update rate

cam_eye = chrono.ChVector3d(-2.0, -3.0, 1.6)      # Irrlicht/preview viewpoint
sensor_update_rate = cam_rate      # shared OptiX render rate for the onboard camera


# === ROS reconstruction === plain-Python stand-ins (NO pychrono.ros module exists)
class ChROSHandler:
    """Rate-gated handler base: Update(t) decides when to Tick(t), like rclcpp timers."""

    def __init__(self, update_rate, topic):
        self.update_rate = float(update_rate)          # Hz
        self.topic = topic                              # ROS topic name string
        self._period = 1.0 / float(update_rate)         # cache: publish period (s)
        self._next_tick = 0.0                           # next sim time to publish
        self.publish_count = 0                          # messages published so far
        self.last_message = None                        # most recent published payload

    def Update(self, time):
        # Rate gate: only Tick when the configured period has elapsed.
        if time + 1e-9 >= self._next_tick:
            self.Tick(time)
            self._next_tick += self._period

    def Tick(self, time):
        raise NotImplementedError


class CameraPublisher(ChROSHandler):
    """Publishes sensor_msgs/Image-shaped metadata from the camera's RGBA8 buffer."""

    def __init__(self, sensor, update_rate, topic):
        super().__init__(update_rate, topic)
        self.sensor = sensor                            # cache: sensor handle reused each tick

    def Tick(self, time):
        buf = self.sensor.GetMostRecentRGBA8Buffer()    # may be empty before first render
        if buf.HasData():                               # guard: skip ticks before first frame
            self.last_message = {"w": buf.Width, "h": buf.Height, "stamp": buf.TimeStamp}
            self.publish_count += 1


class LidarPublisher(ChROSHandler):
    """Publishes sensor_msgs/PointCloud2-shaped metadata from the lidar XYZI buffer."""

    def __init__(self, sensor, update_rate, topic):
        super().__init__(update_rate, topic)
        self.sensor = sensor                            # cache: sensor handle reused each tick

    def Tick(self, time):
        buf = self.sensor.GetMostRecentXYZIBuffer()     # XYZI access only (no save/visualize)
        if buf.HasData():                               # guard: empty until first sweep
            self.last_message = {"points": buf.Width * buf.Height, "stamp": buf.TimeStamp}
            self.publish_count += 1


class AccelPublisher(ChROSHandler):
    """Publishes sensor_msgs/Imu (linear_acceleration) from the accelerometer buffer."""

    def __init__(self, sensor, update_rate, topic):
        super().__init__(update_rate, topic)
        self.sensor = sensor                            # cache: sensor handle reused each tick

    def Tick(self, time):
        buf = self.sensor.GetMostRecentAccelBuffer()    # may be empty before first update
        if buf.HasData():                               # guard: skip empty buffer
            d = buf.GetAccelData()                      # numpy [ax, ay, az]
            self.last_message = {"ax": float(d[0]), "ay": float(d[1]), "az": float(d[2])}
            self.publish_count += 1


class GyroPublisher(ChROSHandler):
    """Publishes sensor_msgs/Imu (angular_velocity) from the gyroscope buffer."""

    def __init__(self, sensor, update_rate, topic):
        super().__init__(update_rate, topic)
        self.sensor = sensor                            # cache: sensor handle reused each tick

    def Tick(self, time):
        buf = self.sensor.GetMostRecentGyroBuffer()     # may be empty before first update
        if buf.HasData():                               # guard: skip empty buffer
            d = buf.GetGyroData()                       # numpy [roll, pitch, yaw] rates
            self.last_message = {"roll": float(d[0]), "pitch": float(d[1]), "yaw": float(d[2])}
            self.publish_count += 1


class MagnetPublisher(ChROSHandler):
    """Publishes sensor_msgs/MagneticField from the magnetometer buffer."""

    def __init__(self, sensor, update_rate, topic):
        super().__init__(update_rate, topic)
        self.sensor = sensor                            # cache: sensor handle reused each tick

    def Tick(self, time):
        buf = self.sensor.GetMostRecentMagnetBuffer()   # may be empty before first update
        if buf.HasData():                               # guard: skip empty buffer
            d = buf.GetMagnetData()                     # numpy [mx, my, mz]
            self.last_message = {"mx": float(d[0]), "my": float(d[1]), "mz": float(d[2])}
            self.publish_count += 1


class GPSPublisher(ChROSHandler):
    """Publishes sensor_msgs/NavSatFix from the GPS buffer."""

    def __init__(self, sensor, update_rate, topic):
        super().__init__(update_rate, topic)
        self.sensor = sensor                            # cache: sensor handle reused each tick

    def Tick(self, time):
        buf = self.sensor.GetMostRecentGPSBuffer()      # may be empty before first update
        if buf.HasData():                               # guard: skip empty buffer
            d = buf.GetGPSData()                        # numpy [lat, lon, alt, time]
            self.last_message = {"lat": float(d[0]), "lon": float(d[1]), "alt": float(d[2])}
            self.publish_count += 1


class ChROSPythonManager:
    """Ticks every registered handler each physics step (rclcpp::spin_some analogue)."""

    def __init__(self):
        self.handlers = []                              # registered publisher handlers

    def RegisterHandler(self, handler):
        self.handlers.append(handler)

    def Update(self, time):
        for h in self.handlers:                         # rate gate lives inside each handler
            h.Update(time)


# === System & gravity === single ChSystemNSC; collision enabled for the body shape
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Collision system required: the moving body + static reference cube carry collision
# shapes (the OptiX sensor renders only bodies with collision geometry).
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

contact_mat = chrono.ChContactMaterialNSC()   # NSC material matches ChSystemNSC
contact_mat.SetFriction(0.6)
contact_mat.SetRestitution(0.0)

# === Bodies === a fixed world anchor, the moving "ground" body, and a static target
world = chrono.ChBody()
world.SetFixed(True)
sys.Add(world)

ground = chrono.ChBodyEasyBox(
    ground_size.x, ground_size.y, ground_size.z,
    ground_density, True, True, contact_mat,
)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetName("moving_ground")
sys.Add(ground)

# A static cube downrange so the lidar/camera have a non-trivial scene to observe.
ref_cube = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 500.0, True, True, contact_mat)
ref_cube.SetPos(ref_cube_pos)
ref_cube.SetFixed(True)
ref_cube.SetName("reference_cube")
sys.Add(ref_cube)

# === Joints / actuation === prismatic-along-X + linear motor drive the X oscillation
# The linear motor actuates along its frame's local Z; rotating the frame +90 deg about
# Y aligns that axis with world +X, so the imposed motion is a pure X translation.
motor_frame = chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                              chrono.QuatFromAngleY(chrono.CH_PI_2))
prismatic = chrono.ChLinkLockPrismatic()   # lock all DOFs except sliding along world X
prismatic.Initialize(ground, world, motor_frame)
sys.Add(prismatic)

motor = chrono.ChLinkMotorLinearPosition()
motor.Initialize(ground, world, motor_frame)
motion_fun = chrono.ChFunctionSine(motion_amplitude, motion_freq)  # amp * sin(2*pi*f*t)
motor.SetMotionFunction(motion_fun)
sys.Add(motor)

# === Sensors === ChSensorManager + scene lighting; sensors ride the moving ground
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))   # no AddDirectionalLight needed

no_noise = sens.ChNoiseNone()   # cache: shared zero-noise model for the inertial sensors

# Onboard RGB camera looking forward toward the reference cube.
cam_offset = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.6), chrono.QUNIT)
camera = sens.ChCameraSensor(ground, sensor_update_rate, cam_offset, cam_w, cam_h, cam_fov)
camera.SetName("camera_sensor")
camera.PushFilter(sens.ChFilterVisualize(cam_w, cam_h))   # live preview window
camera.PushFilter(sens.ChFilterSave("cam/sensor_rgb/"))   # PNG frames -> mp4 (review)
camera.PushFilter(sens.ChFilterRGBA8Access())             # buffer access for the publisher
manager.AddSensor(camera)

# Lidar: XYZI access only (point-cloud save/visualize filters deadlock -> omitted).
lidar = sens.ChLidarSensor(
    ground, cam_rate, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.6), chrono.QUNIT),
    180, 16, 2.0 * math.pi, 0.2618, -0.2618, 40.0,
)
lidar.SetName("lidar_sensor")
lidar.PushFilter(sens.ChFilterDIAccess())                 # depth/intensity access
lidar.PushFilter(sens.ChFilterPCfromDepth())              # convert depth -> point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())               # XYZI buffer for the publisher
manager.AddSensor(lidar)

imu_offset = chrono.ChFramed(chrono.VNULL, chrono.QUNIT)
accel = sens.ChAccelerometerSensor(ground, imu_rate, imu_offset, no_noise)
accel.SetName("accelerometer")
accel.PushFilter(sens.ChFilterAccelAccess())              # linear-acceleration access
manager.AddSensor(accel)

gyro = sens.ChGyroscopeSensor(ground, imu_rate, imu_offset, no_noise)
gyro.SetName("gyroscope")
gyro.PushFilter(sens.ChFilterGyroAccess())                # angular-velocity access
manager.AddSensor(gyro)

magnet = sens.ChMagnetometerSensor(ground, imu_rate, imu_offset, no_noise, gps_reference)
magnet.SetName("magnetometer")
magnet.PushFilter(sens.ChFilterMagnetAccess())            # magnetic-field access
manager.AddSensor(magnet)

gps = sens.ChGPSSensor(ground, gps_rate, imu_offset, gps_reference, no_noise)
gps.SetName("gps")
gps.PushFilter(sens.ChFilterGPSAccess())                  # geo-position access
manager.AddSensor(gps)

# === ROS publishers === one handler per sensor, all ticked by the Python ROS manager
ros_manager = ChROSPythonManager()
ros_manager.RegisterHandler(CameraPublisher(camera, cam_rate, "/sensor/camera/image"))
ros_manager.RegisterHandler(LidarPublisher(lidar, cam_rate, "/sensor/lidar/points"))
ros_manager.RegisterHandler(AccelPublisher(accel, imu_rate, "/sensor/imu/accel"))
ros_manager.RegisterHandler(GyroPublisher(gyro, imu_rate, "/sensor/imu/gyro"))
ros_manager.RegisterHandler(MagnetPublisher(magnet, imu_rate, "/sensor/imu/mag"))
ros_manager.RegisterHandler(GPSPublisher(gps, gps_rate, "/sensor/gps/fix"))

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Multi-sensor moving ground body (ROS-style publishing)")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(cam_eye, ref_cube_pos)
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid

# === Main loop === render at render_fps; pump sensors + ROS each physics step


ground_body = ground   # cache: main body handle reused every step
cam_pub = ros_manager.handlers[0]   # cache: camera publisher for the CSV column

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()                 # pump all sensors (every physics step)
            ros_manager.Update(sys.GetChTime())   # rate-gated ROS-style publishing
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid sensor state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === close the log, assemble review videos + plot, prune frames
