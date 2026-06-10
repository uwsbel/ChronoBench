"""Sensor scene with a 2D lidar published over a ROS-style handler stack.

Models a small NSC scene (a fixed ground plane plus three colliding rigid
boxes that settle under gravity) observed by sensors mounted on a fixed mast
body. A horizontal-plane (2D) lidar sweeps the scene and its range scan is
relayed to a ROS-shaped publisher topic; an RGB camera supplies the review
imagery. The bodies make contact with the ground and each other, so the scene
uses a Bullet collision system.

System type: NSC (ChSystemNSC).
Main bodies: fixed ground, fixed sensor mast, three free boxes.
Sensors: a 2D ChLidarSensor (h=1 row) and an RGB ChCameraSensor on the mast.
ROS reconstruction: PyChrono 9.0.1 ships NO pychrono.ros module, so the ROS
graph is reconstructed in plain Python — a ChROSHandler base whose Update()
is rate-gated before calling Tick(), a lidar-scan publisher handler that reads
the most-recent XYZI buffer, and a ChROSPythonManager that ticks every handler
once per simulation step.
Expected behavior: the boxes drop, contact the ground/each other, and settle;
the lidar scan and camera frames update continuously and the lidar handler
"publishes" a populated scan to its topic.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === geometry / physics / sensor configuration (no bare literals downstream)
time_step = 2e-3                       # s, integration step
sim_end = 6.0                          # s, total simulated time
render_fps = 30.0                      # Hz, Irrlicht review cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

ground_size = (12.0, 12.0, 0.4)        # full extents of the ground slab (m)
ground_top_z = 0.0                     # top surface of the ground at z = 0
box_size = 0.6                         # cube edge length (m)
box_density = 250.0                    # kg/m^3
box_drop_z = 1.6                       # initial center height of the boxes (m)
box_xs = (-1.2, 0.0, 1.2)              # x spawn positions, footprints kept clear

mast_pos = chrono.ChVector3d(0.0, -5.0, 1.0)   # fixed sensor mast location
scan_target = chrono.ChVector3d(0.0, 0.0, 0.5)  # where the sensors look

# 2D lidar configuration: a single horizontal scan row sweeping a wide arc.
lidar_update_rate = 25.0               # Hz, scan rate
lidar_h_samples = 360                  # horizontal samples across the arc
lidar_v_samples = 1                    # 2D lidar -> exactly one vertical row
lidar_hfov = 2.0 * math.pi             # full 360 deg horizontal field of view
lidar_max_vert = 0.0                   # single horizontal plane (rad)
lidar_min_vert = 0.0                   # single horizontal plane (rad)
lidar_max_distance = 40.0              # m, maximum range
lidar_topic = "~/output/lidar2d/data/scan"   # ROS-style scan topic

cam_update_rate = float(render_fps)    # Hz, review camera rate
cam_w, cam_h = 1280, 720               # review image resolution
cam_hfov = 1.408                       # rad, horizontal field of view


# === ROS-shaped handler stack (no pychrono.ros module exists in 9.0.1) ===
# Reconstructed in plain Python: a rate-gated handler base, a lidar-scan
# publisher reading the most-recent buffer, and a manager that ticks each step.
class ChROSHandler:
    """Base ROS handler: Update() is rate-gated, then defers to Tick()."""

    def __init__(self, update_rate):
        self._period = 1.0 / update_rate if update_rate > 0.0 else 0.0
        self._next_time = 0.0

    def Update(self, sim_time):
        # Rate gate: only publish when the handler's period has elapsed.
        if sim_time + 1e-9 < self._next_time:
            return
        self._next_time = sim_time + self._period
        self.Tick(sim_time)

    def Tick(self, sim_time):
        raise NotImplementedError


class ChROSLidar2DHandler(ChROSHandler):
    """Publishes the most-recent 2D lidar scan to a ROS-style scan topic."""

    def __init__(self, lidar_sensor, topic, update_rate):
        super().__init__(update_rate)
        self._lidar = lidar_sensor          # cache: sensor handle, reused every tick
        self.topic = topic
        self.last_ranges = []               # most recently published scan ranges
        self.publish_count = 0

    def Tick(self, sim_time):
        # Read the most-recent buffer; it is empty until the lidar's first scan.
        buf = self._lidar.GetMostRecentXYZIBuffer()
        if not buf.HasData():               # guard: skip ticks before first scan
            return
        pts = buf.GetXYZIData()             # numpy array (H, W, 4): x, y, z, intensity
        flat = pts.reshape(-1, 4)           # one row per beam return
        ranges = [math.sqrt(float(r[0]) ** 2 + float(r[1]) ** 2) for r in flat]
        self.last_ranges = ranges           # "published" message payload
        self.publish_count += 1


class ChROSPythonManager:
    """Owns the ROS handlers and ticks each one every simulation step."""

    def __init__(self):
        self._handlers = []

    def RegisterHandler(self, handler):
        self._handlers.append(handler)

    def Update(self, sim_time):
        for handler in self._handlers:
            handler.Update(sim_time)


def main():
    # === System & gravity === NSC system with Bullet collision (bodies contact)
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # bodies collide

    contact_mat = chrono.ChContactMaterialNSC()
    contact_mat.SetFriction(0.6)
    contact_mat.SetRestitution(0.0)

    # === Bodies === fixed ground, fixed sensor mast, three dropping boxes
    ground = chrono.ChBodyEasyBox(ground_size[0], ground_size[1], ground_size[2],
                                  1000.0, True, True, contact_mat)
    ground.SetPos(chrono.ChVector3d(0, 0, ground_top_z - ground_size[2] / 2.0))
    ground.SetFixed(True)
    sys.Add(ground)

    mast = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 500.0, True, True, contact_mat)
    mast.SetPos(mast_pos)
    mast.SetFixed(True)
    sys.Add(mast)

    box_colors = (chrono.ChColor(0.8, 0.2, 0.2),
                  chrono.ChColor(0.2, 0.8, 0.2),
                  chrono.ChColor(0.2, 0.2, 0.8))
    for i, bx in enumerate(box_xs):
        box = chrono.ChBodyEasyBox(box_size, box_size, box_size,
                                   box_density, True, True, contact_mat)
        box.SetPos(chrono.ChVector3d(bx, 0.0, box_drop_z + i * 0.05))
        box.GetVisualShape(0).SetColor(box_colors[i])
        sys.Add(box)

    # === Sensor manager & lighting === drives the OptiX lidar + camera renders
    manager = sens.ChSensorManager(sys)
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                                chrono.ChColor(1.0, 1.0, 1.0), 500.0)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

    # === 2D lidar sensor === single horizontal row, 360 deg sweep, named for viz
    lidar = sens.ChLidarSensor(
        mast,                                          # rides on the fixed mast
        lidar_update_rate,
        chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        lidar_h_samples, lidar_v_samples,
        lidar_hfov, lidar_max_vert, lidar_min_vert, lidar_max_distance,
        sens.LidarBeamShape_RECTANGULAR, 1, 0.003, 0.003,
        sens.LidarReturnMode_STRONGEST_RETURN, 1e-3,
    )
    lidar.SetName("lidar2d")                            # named for visualization
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(0.0)
    # Lidar uses DI/PCfromDepth/XYZI access filters ONLY — no point-cloud
    # save/visualize, which would deadlock with the Irrlicht window.
    lidar.PushFilter(sens.ChFilterDIAccess())          # depth/intensity access
    lidar.PushFilter(sens.ChFilterPCfromDepth())       # depth -> point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())        # xyz + intensity access
    manager.AddSensor(lidar)

    # === Review camera sensor === RGB frames for the review video (OptiX render)
    cam = sens.ChCameraSensor(
        mast, cam_update_rate,
        chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
        cam_w, cam_h, cam_hfov,
    )
    cam.SetName("review_cam")
    cam.PushFilter(sens.ChFilterRGBA8Access())         # frame-buffer access
    cam.PushFilter(sens.ChFilterVisualize(cam_w, cam_h))   # live preview window
    cam.PushFilter(sens.ChFilterSave("cam/review_cam/"))   # PNG frames -> mp4
    manager.AddSensor(cam)

    # Aim the camera body so it looks from the mast toward the scene center.
    forward = (scan_target - mast_pos).GetNormalized()
    look_at_quat = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), forward)
    mast.SetRot(look_at_quat)

    # === ROS handler stack === reconstructed ROS graph publishing the lidar scan
    ros_manager = ChROSPythonManager()
    lidar_handler = ChROSLidar2DHandler(lidar, lidar_topic, lidar_update_rate)
    ros_manager.RegisterHandler(lidar_handler)

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("2D Lidar Sensor Scene with ROS Scan Publisher")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(6, -8, 4), scan_target)
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))


    # === Main loop === render at cadence; pump sensors + ROS handlers each step
    boxes = [b for b in sys.GetBodies() if not b.IsFixed()]   # cache: free bodies
    frame = 0
    try:
        while vis.Run() and sys.GetChTime() < sim_end:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(render_every):
                manager.Update()                       # pump sensors every step
                sim_time = sys.GetChTime()
                ros_manager.Update(sim_time)           # tick ROS handlers (rate-gated)
                sys.DoStepDynamics(time_step)
                if sys.GetChTime() >= sim_end:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Always release the lidar scan payload so a mid-run abort leaves no
        # dangling reference to the most-recent buffer.
        lidar_handler.last_ranges = []


if __name__ == "__main__":
    main()
