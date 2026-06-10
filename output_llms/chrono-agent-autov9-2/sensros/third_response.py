"""Sensor scene with ROS-shaped publishing of a camera and a 2D lidar.

Model
-----
A single fixed triangle-mesh body (the Chrono "vehicle" mesh) sits at the origin
as the sensing target. Two range/vision sensors observe it through a
``ChSensorManager``:

* an RGB ``ChCameraSensor`` (with ``ChFilterRGBA8Access`` for buffer reads and the
  standard ``ChFilterVisualize`` / ``ChFilterSave`` filters), and
* a 2D ``ChLidarSensor`` (single scan row) whose depth/intensity returns are made
  available through ``ChFilterDIAccess`` -> ``ChFilterPCfromDepth`` ->
  ``ChFilterXYZIAccess``. The lidar deliberately carries NO point-cloud save or
  visualize filter: a point-cloud visualize filter deadlocks against the Irrlicht
  window in this build, so only access filters are used on the lidar.

System type
-----------
``ChSystemNSC``. This is a purely static sensor-test scene: the only body is a
fixed mesh with no collision geometry and nothing moves or contacts, so the Bullet
collision system is intentionally not enabled (there is no contact to resolve).

ROS reconstruction
-------------------
There is no ROS Python module in this build, so the ROS publishing pipeline is
reconstructed in plain Python that mirrors the Chrono-ROS bridge: a
``ChROSHandler`` base rate-gates ``Update(time)`` and calls ``Tick(time)`` only when
its publish period has elapsed; per-sensor publisher handlers read the most-recent
sensor buffer and "publish" (here, count + summarize) it; and a
``ChROSPythonManager`` advances every handler once per simulation step and reports
whether the update succeeded. The main loop checks that manager update status and
stops early if any handler fails, exactly as a ROS-driven node would.

Expected behavior
------------------
The mesh stays fixed; the camera and lidar tick at their own rates; the camera
review video shows the static mesh from a fixed viewpoint; the lidar access
buffers fill with finite depth/intensity returns once the sensors warm up, and the
ROS-shaped manager keeps publishing without error for the whole run.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === geometry / timing / sensor rates (no bare literals downstream)
time_step = 2.0e-3            # s, integration step
sim_end = 6.0                # s, total simulated time
render_fps = 30.0            # Hz, Irrlicht review cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

cam_update_rate = 15.0       # Hz, camera sensor tick
lidar_update_rate = 10.0     # Hz, 2D lidar sensor tick
cam_pub_rate = 10.0          # Hz, ROS camera publisher
lidar_pub_rate = 10.0        # Hz, ROS lidar publisher

img_w, img_h = 1280, 720     # camera resolution
cam_hfov = 1.408             # rad, camera horizontal FOV

# 2D lidar: one scan row spanning a wide horizontal sector.
lidar_w = 360                # horizontal samples
lidar_h = 1                  # single row -> "2D" lidar
lidar_hfov = 2.0 * math.pi   # rad, full horizontal sweep
lidar_max_v = 0.0            # rad, vertical max angle (flat scan)
lidar_min_v = 0.0            # rad, vertical min angle (flat scan)
lidar_max_dist = 100.0       # m, max range

cam_eye = chrono.ChVector3d(-6.0, 0.0, 2.5)     # camera position (world)
sensor_target = chrono.ChVector3d(0.0, 0.0, 0.5)  # look-at: mesh center-ish
lidar_pos = chrono.ChVector3d(0.0, 0.0, 1.0)    # lidar position (world)

# === System & gravity === NSC system; static sensor test, no contact/collision
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Bodies === one fixed triangle-mesh body as the sensing target
mesh_shape = chrono.ChVisualShapeTriangleMesh()
mesh_shape.SetMesh(
    chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
        chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), True, True
    )
)
mesh_shape.SetMutable(False)

mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.SetFixed(True)                 # static target: never integrates motion
mesh_body.AddVisualShape(mesh_shape, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.Add(mesh_body)                       # add the mesh body to the simulation system

# === Sensors === ChSensorManager drives both the camera and the 2D lidar
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1.0, 1.0, 1.0), 500.0
)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# Camera rides on a fixed helper body, aimed from cam_eye toward the mesh center.
cam_body = chrono.ChBody()
cam_body.SetFixed(True)
cam_body.SetPos(cam_eye)
sys.Add(cam_body)

forward = (sensor_target - cam_eye).GetNormalized()  # precomputed once: view dir
cam_look = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), forward)

camera = sens.ChCameraSensor(
    cam_body,                                    # body the camera rides on
    cam_update_rate,                             # Hz
    chrono.ChFramed(chrono.VNULL, cam_look),     # offset frame: look-at the mesh
    img_w, img_h,                                # resolution
    cam_hfov,                                    # horizontal FOV (rad)
)
camera.PushFilter(sens.ChFilterVisualize(img_w, img_h))  # live preview window
camera.PushFilter(sens.ChFilterSave("cam/sensor_rgb/"))  # PNG frames -> mp4 by RUN stage
camera.PushFilter(sens.ChFilterRGBA8Access())            # RGBA frame-buffer access
manager.AddSensor(camera)

# 2D lidar on its own fixed helper body, scanning the horizontal plane.
lidar_body = chrono.ChBody()
lidar_body.SetFixed(True)
lidar_body.SetPos(lidar_pos)
sys.Add(lidar_body)

lidar = sens.ChLidarSensor(
    lidar_body,                                  # body the lidar rides on
    lidar_update_rate,                           # Hz
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT), # offset frame
    lidar_w, lidar_h,                            # horizontal samples x 1 row (2D)
    lidar_hfov,                                  # horizontal FOV (rad)
    lidar_max_v, lidar_min_v,                    # vertical max/min angle (flat)
    lidar_max_dist,                              # max range (m)
)
# Access-only filter chain for the 2D lidar: depth/intensity, then point cloud
# from depth, then XYZI access. NO point-cloud save/visualize (Irrlicht deadlock).
lidar.PushFilter(sens.ChFilterDIAccess())        # depth + intensity buffer access
lidar.PushFilter(sens.ChFilterPCfromDepth())     # convert depth -> point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())      # XYZI point-cloud access
manager.AddSensor(lidar)


# === ROS reconstruction === plain-Python mirror of the Chrono-ROS bridge
class ChROSHandler:
    """Base ROS handler: rate-gate Update(time) and Tick(time) at update_rate Hz."""

    def __init__(self, update_rate):
        self._period = 1.0 / update_rate if update_rate > 0 else 0.0
        self._next_pub = 0.0

    def Update(self, time):
        # Publish only when the handler's period has elapsed (ROS rate gating).
        if time + 1e-12 < self._next_pub:
            return True
        self._next_pub = time + self._period
        return self.Tick(time)

    def Tick(self, time):
        raise NotImplementedError


class ChROSCameraHandler(ChROSHandler):
    """Reads the camera's most-recent RGBA8 buffer and 'publishes' it."""

    def __init__(self, update_rate, camera_sensor):
        super().__init__(update_rate)
        self._cam = camera_sensor              # cache: handler keeps its sensor ref
        self.published = 0
        self.last_pixels = 0

    def Tick(self, time):
        buf = self._cam.GetMostRecentRGBA8Buffer()  # empty before first sensor tick
        if buf.HasData():                            # guard: skip unfilled buffers
            self.last_pixels = int(buf.Width) * int(buf.Height)
            self.published += 1
        return True


class ChROSLidarHandler(ChROSHandler):
    """Reads the 2D lidar's most-recent XYZI buffer and 'publishes' it."""

    def __init__(self, update_rate, lidar_sensor):
        super().__init__(update_rate)
        self._lidar = lidar_sensor             # cache: handler keeps its sensor ref
        self.published = 0
        self.last_points = 0

    def Tick(self, time):
        buf = self._lidar.GetMostRecentXYZIBuffer()  # empty before first lidar tick
        if buf.HasData():                            # guard: skip unfilled buffers
            self.last_points = int(buf.Width) * int(buf.Height)
            self.published += 1
        return True


class ChROSPythonManager:
    """Ticks every registered handler once per step; reports overall success."""

    def __init__(self):
        self._handlers = []                    # cache: handler list, reused each step

    def RegisterHandler(self, handler):
        self._handlers.append(handler)

    def Update(self, time):
        # Returns False if any handler fails -> the main loop stops early.
        ok = True
        for h in self._handlers:
            ok = h.Update(time) and ok
        return ok


ros_manager = ChROSPythonManager()
cam_handler = ChROSCameraHandler(cam_pub_rate, camera)
lidar_handler = ChROSLidarHandler(lidar_pub_rate, lidar)
ros_manager.RegisterHandler(cam_handler)
ros_manager.RegisterHandler(lidar_handler)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Sensor scene: camera + 2D lidar (ROS-shaped publishing)")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(cam_eye, sensor_target)
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics + sensors in inner batch

try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()                  # pump sensors every physics step
            t = sys.GetChTime()
            if not ros_manager.Update(t):      # ROS-shaped publish; stop if it fails
                print("ROS manager update failed; stopping simulation loop.")
                raise RuntimeError("ROS handler update returned failure")
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
