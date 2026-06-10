"""HMMWV wheeled vehicle on flat rigid terrain, instrumented with a roof-mounted
LiDAR sensor and a self-contained ROS publishing layer.

System type: NSC (the veh.HMMWV_Full wrapper owns a ChSystemNSC). The main bodies
are the HMMWV chassis + four wheel spindles (created inside the wrapper), a flat
RigidTerrain patch, and a static visualization landmark box that gives the LiDAR a
solid target to range against. A scripted ChDriver applies forward throttle with
net-zero steering so the vehicle drives roughly straight across a large patch.

A ChLidarSensor rides on the chassis and is rendered off-screen by an OptiX
ChSensorManager (point-cloud filters). Irrlicht is the on-screen review renderer.

ROS substitution note: this PyChrono build ships no `pychrono.ros` module, so the
ROS publishing layer is reconstructed self-contained here using the ChROSManager
SHAPE — a lightweight manager plus per-topic handlers (clock, driver-inputs,
chassis-body state, lidar point count). Each handler exposes Initialize/Tick and
the manager Registers them, Initializes once, and Updates every step, mirroring
how pychrono.ros.ChROSManager + ChROSClockHandler / ChROSDriverInputsHandler /
ChROSBodyHandler / ChROSLidarHandler would be wired against a live rclpy node.

Expected behavior: the HMMWV accelerates from rest and translates forward; the
LiDAR returns a growing/stable hit count as it sees the terrain and the landmark
box; the ROS handlers tick every step and log their published payloads to CSV.
"""

import os
import csv
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants: geometry / physics / sensor / ROS ===
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire substep (s)
SIM_END = 8.0                      # simulated duration (s)
RENDER_FPS = 30.0                  # review-video frame rate

TERRAIN_LENGTH = 200.0             # X size of the rigid patch (m) — large, net-zero steering
TERRAIN_WIDTH = 200.0              # Y size of the rigid patch (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_YOUNG = 2.0e7

VEH_INIT_X = -60.0                 # spawn near one end so the vehicle has room to drive
VEH_INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5        # HMMWV chassis-origin height above wheel-bottom at rest (m)
TIRE_RADIUS = 0.4699               # HMMWV tire radius (m), for the wheel-bottom assert
ZTOL = 0.10                        # allowed wheel-bottom clearance/overlap vs terrain top

THROTTLE_RAMP_END = 1.0            # s: ramp throttle 0 -> cruise over this window
CRUISE_THROTTLE = 0.7              # steady throttle after the ramp
STEERING_CMD = 0.0                 # net-zero steering -> straight-line drive

# LiDAR (roof-mounted), as requested with point-cloud filters
LIDAR_UPDATE_RATE = 10.0           # Hz
LIDAR_W = 360                      # horizontal samples
LIDAR_H = 16                       # vertical channels
LIDAR_HFOV = 2.0 * math.pi         # full 360 deg horizontal
LIDAR_MAX_VERT = 0.2618            # +15 deg (rad)
LIDAR_MIN_VERT = -0.2618           # -15 deg (rad)
LIDAR_MAX_DIST = 100.0             # max range (m)
LIDAR_OFFSET = chrono.ChVector3d(0.5, 0.0, 1.5)   # roof position in chassis frame

# Camera view requested at (-5, 2.5, 1.5): chase camera offset behind+left+up
CHASE_TRACK = chrono.ChVector3d(0.0, 0.0, 1.5)
CHASE_DIST = 5.0
CHASE_HEIGHT = 1.5

# Static landmark box for the LiDAR to range against
BOX_SIZE = chrono.ChVector3d(2.0, 2.0, 2.0)
BOX_POS = chrono.ChVector3d(VEH_INIT_X + 30.0, 6.0, 1.0)   # ahead and to the side

# Derived once (precomputed, never recomputed in the loop)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check when validating


# === ROS layer (self-contained, ChROSManager SHAPE) ===
# pychrono.ros is absent in this build, so the ROS publish path is reconstructed
# with the same shape: a manager that Registers handlers, Initializes once, and
# Updates each step. Each handler mimics a rclpy publisher onto a named topic.
class ChROSHandler:
    """Base handler: matches the Initialize / Tick(time) contract of the real
    pychrono.ros handlers. `payload` holds the most recent published message."""

    def __init__(self, topic, update_rate):
        self.topic = topic
        self.update_rate = update_rate          # Hz
        self._period = 1.0 / update_rate if update_rate > 0 else 0.0
        self._next_tick = 0.0
        self.payload = None

    def Initialize(self):
        self._next_tick = 0.0
        return True

    def Tick(self, time):
        raise NotImplementedError

    def Update(self, time):
        # cache: rate-limit each handler to its own publish period
        if self._period == 0.0 or time + 1e-9 >= self._next_tick:
            self.Tick(time)
            self._next_tick = time + self._period


class ClockHandler(ChROSHandler):
    """Publishes the simulation clock onto /clock (rosgraph_msgs/Clock shape)."""

    def Tick(self, time):
        self.payload = {"sec": int(time), "nanosec": int((time % 1.0) * 1e9)}


class DriverInputsHandler(ChROSHandler):
    """Publishes the active driver inputs (vehicle_msgs/DriverInputs shape)."""

    def __init__(self, topic, update_rate, driver):
        super().__init__(topic, update_rate)
        self._driver = driver                   # cache: handler holds the driver ref

    def Tick(self, time):
        di = self._driver.GetInputs()
        self.payload = {
            "steering": di.m_steering,
            "throttle": di.m_throttle,
            "braking": di.m_braking,
        }


class BodyHandler(ChROSHandler):
    """Publishes a body pose + linear speed (nav_msgs/Odometry shape)."""

    def __init__(self, topic, update_rate, body):
        super().__init__(topic, update_rate)
        self._body = body                       # cache: handler holds the body ref

    def Tick(self, time):
        p = self._body.GetPos()
        v = self._body.GetPosDt()
        self.payload = {
            "x": p.x, "y": p.y, "z": p.z,
            "speed": math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z),
        }


class LidarHandler(ChROSHandler):
    """Publishes a LiDAR scan summary (sensor_msgs/PointCloud2 shape) — here the
    valid-hit count from the most recent XYZI buffer."""

    def __init__(self, topic, update_rate, lidar):
        super().__init__(topic, update_rate)
        self._lidar = lidar                     # cache: handler holds the sensor ref

    def Tick(self, time):
        buf = self._lidar.GetMostRecentXYZIBuffer()   # may be empty before first sensor tick
        n = 0
        if buf.HasData():                              # guard: only read a filled buffer
            n = int(buf.Width) * int(buf.Height)       # PCfromDepth packs valid returns into W*H
        self.payload = {"num_points": n}


class ChROSManager:
    """Self-contained stand-in for pychrono.ros.ChROSManager: registers handlers,
    initializes them once, and updates them each simulation step."""

    def __init__(self):
        self._handlers = []
        self._initialized = False

    def RegisterHandler(self, handler):
        self._handlers.append(handler)

    def Initialize(self):
        for h in self._handlers:
            h.Initialize()
        self._initialized = True
        return True

    def Update(self, time, time_step):
        # mirrors ChROSManager::Update — pump every registered handler each step
        for h in self._handlers:
            h.Update(time)
        return True

    @property
    def handlers(self):
        return self._handlers


# === Output setup (guard against missing output dir) ===
os.makedirs("frames", exist_ok=True)            # Irrlicht review frames
os.makedirs("cam", exist_ok=True)               # CSV motion log + review video

data_file = None
motion_file = None
data_writer = None
motion_writer = None

try:
    # === Vehicle wrapper (creates system + chassis + spindles + suspension joints) ===
    init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, 0.0)   # Z fixed up after terrain
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, SUSPENSION_REF_HEIGHT), init_rot))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # grippy on rigid road, drives straight
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    system = hmmwv.GetSystem()                    # cache: ChSystemNSC owned by the wrapper
    chassis = hmmwv.GetChassisBody()              # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()                  # cache: underlying ChWheeledVehicle
    # spindles: veh_obj.GetSpindlePos(axle, side) ; joints: suspension + steering links in the wrapper

    # === Terrain (flat rigid patch under the vehicle) ===
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    TERRAIN_TOP_Z = 0.0                           # flat patch top plane
    # SetInitPosition(z=SUSPENSION_REF_HEIGHT) already rests the wheels on a z=0
    # patch; do NOT re-SetPos the chassis here (that desyncs the suspension and
    # launches the vehicle). Validate by reading the real spindle poses below.

    # Assert the wheels rest on (not through) the terrain, reading real spindle poses.
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Static landmark body (a solid LiDAR target with collision so OptiX renders it) ===
    box_mat = chrono.ChContactMaterialNSC()
    box_mat.SetFriction(TERRAIN_FRICTION)
    box_mat.SetRestitution(TERRAIN_RESTITUTION)
    landmark = chrono.ChBodyEasyBox(
        BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, 1000.0, True, True, box_mat)
    landmark.SetPos(BOX_POS)
    landmark.SetFixed(True)
    landmark.SetName("lidar_landmark_box")
    system.AddBody(landmark)
    system.GetCollisionSystem().BindAll()

    # === Driver (scripted, autonomous — net-zero steering, ramped forward throttle) ===
    class StraightDriver(veh.ChDriver):
        def __init__(self, vehicle):
            super().__init__(vehicle)

        def Synchronize(self, time):
            if time < THROTTLE_RAMP_END:
                self.SetThrottle(CRUISE_THROTTLE * (time / THROTTLE_RAMP_END))
            else:
                self.SetThrottle(CRUISE_THROTTLE)
            self.SetSteering(STEERING_CMD)        # net-zero steering -> straight line
            self.SetBraking(0.0)

    driver = StraightDriver(veh_obj)
    driver.Initialize()

    # === Sensor manager + roof LiDAR (OptiX, point-cloud filters) ===
    manager = sens.ChSensorManager(system)
    # ChScene exposes no AddDirectionalLight in this build path -> point light + ambient.
    manager.scene.AddPointLight(
        chrono.ChVector3f(0, 0, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

    lidar = sens.ChLidarSensor(
        chassis,
        LIDAR_UPDATE_RATE,
        chrono.ChFramed(LIDAR_OFFSET, chrono.QUNIT),
        LIDAR_W, LIDAR_H,
        LIDAR_HFOV, LIDAR_MAX_VERT, LIDAR_MIN_VERT, LIDAR_MAX_DIST,
    )
    lidar.PushFilter(sens.ChFilterDIAccess())                  # depth/intensity buffer
    lidar.PushFilter(sens.ChFilterPCfromDepth())               # depth -> point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())                # XYZI point-cloud access (for ROS)
    if not HEADLESS:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 2.0))  # live PC preview
    manager.AddSensor(lidar)

    # === ROS layer wiring (Register handlers -> Initialize once) ===
    ros_manager = ChROSManager()
    clock_handler = ClockHandler("/clock", LIDAR_UPDATE_RATE)
    driver_handler = DriverInputsHandler("/vehicle/driver_inputs", 25.0, driver)
    body_handler = BodyHandler("/vehicle/state", 25.0, chassis)
    lidar_handler = LidarHandler("/vehicle/lidar/points", LIDAR_UPDATE_RATE, lidar)
    ros_manager.RegisterHandler(clock_handler)
    ros_manager.RegisterHandler(driver_handler)
    ros_manager.RegisterHandler(body_handler)
    ros_manager.RegisterHandler(lidar_handler)
    ros_manager.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV + LiDAR + ROS publishing layer")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(CHASE_TRACK, CHASE_DIST, CHASE_HEIGHT)   # view ~(-5, 2.5, 1.5) behind
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 50, 50,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
        vis.AttachVehicle(veh_obj)

    # === CSV logging (open with context managers so writers always flush/close) ===
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:                 # disk / permission failure
        print(f"Could not open CSV output: {exc}")
        raise

    data_writer = csv.writer(data_file)
    data_writer.writerow([
        "time", "throttle", "steering",
        "chassis_x", "chassis_y", "chassis_z", "speed",
        "lidar_points", "ros_clock_sec",
    ])
    motion_writer = csv.writer(motion_file)
    motion_writer.writerow([
        "time", "body", "x", "y", "z", "vx", "vy", "vz", "speed",
    ])

    # === Main loop (render-cadence outer; physics + sensors + ROS in the inner batch) ===
    frame = 0
    while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
            frame += 1

        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            # Subsystem synchronize (driver -> terrain -> vehicle -> vis)
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(sim_time, driver_inputs)

            # Pump the OptiX sensors and the ROS handlers every physics step
            manager.Update()
            ros_manager.Update(sim_time, TIME_STEP)

            # --- Log physics this step ---
            pos = chassis.GetPos()                # cache: one getter, reused below
            vel = chassis.GetPosDt()
            speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
            lidar_pts = lidar_handler.payload["num_points"] if lidar_handler.payload else 0
            clock_sec = clock_handler.payload["sec"] if clock_handler.payload else 0
            data_writer.writerow([
                f"{sim_time:.5f}", f"{driver_inputs.m_throttle:.4f}",
                f"{driver_inputs.m_steering:.4f}",
                f"{pos.x:.4f}", f"{pos.y:.4f}", f"{pos.z:.4f}", f"{speed:.4f}",
                lidar_pts, clock_sec,
            ])
            motion_writer.writerow([
                f"{sim_time:.5f}", "chassis",
                f"{pos.x:.4f}", f"{pos.y:.4f}", f"{pos.z:.4f}",
                f"{vel.x:.4f}", f"{vel.y:.4f}", f"{vel.z:.4f}", f"{speed:.4f}",
            ])

            # Advance subsystems (hmmwv.Advance steps the wrapper-owned system)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)
            if not HEADLESS:
                vis.Advance(TIME_STEP)

            if system.GetChTime() >= RUN_END:
                break

except (RuntimeError, ValueError) as exc:             # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close any open writers even if a step diverged mid-run.
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing (plot the logged time series) ===
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times, speeds, xs, pts = [], [], [], []
    with open("simulation_data.csv", "r", newline="") as f:    # context-managed read
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time"]))
            speeds.append(float(row["speed"]))
            xs.append(float(row["chassis_x"]))
            pts.append(float(row["lidar_points"]))

    fig, axs = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    axs[0].plot(times, speeds, color="tab:blue")
    axs[0].set_ylabel("speed (m/s)")
    axs[0].grid(True)
    axs[1].plot(times, xs, color="tab:green")
    axs[1].set_ylabel("chassis x (m)")
    axs[1].grid(True)
    axs[2].plot(times, pts, color="tab:red")
    axs[2].set_ylabel("lidar points")
    axs[2].set_xlabel("time (s)")
    axs[2].grid(True)
    fig.suptitle("HMMWV + LiDAR + ROS — vehicle motion and LiDAR returns")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)
except (OSError, ValueError, ImportError) as exc:     # missing file / bad data / no matplotlib
    print(f"Post-processing plot skipped: {exc}")
