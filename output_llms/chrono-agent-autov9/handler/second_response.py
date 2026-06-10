"""ROS-handler publisher demo on a real PyChrono rigid-body scene (Irrlicht).

This script models a single dynamic rigid body (a unit-density box) that is
released under gravity and bounces on a fixed ground plate inside a
ChSystemNSC (Non-Smooth Contact) world. Around that real physics it builds a
self-contained, dependency-free *ROS-shaped* publishing framework.

NOTE ON THE ROS MODULE SUBSTITUTION
-----------------------------------
This PyChrono 9.0.1 build ships NO `pychrono.ros` module (the available
submodules are core/fea/fsi/irrlicht/robot/sensor/vehicle/cascade/parsers/
postprocess/pardisomkl). The requested ROS architecture is therefore
reconstructed here as plain Python classes that mirror the SHAPE of the real
`pychrono.ros` API exactly:
  * ChROSClockHandler  -> publishes simulation time on topic "/clock"
  * ChROSBodyHandler   -> publishes the tracked body pose ("/body/pose") and
                          twist ("/body/twist")
  * ChROSTFHandler     -> publishes a TF transform world -> body on "/tf"
  * MyCustomHandler    -> a custom handler that publishes a String message on a
                          user topic; it owns a `message` attribute equal to
                          "Hello, world! At time: " and, each Tick, publishes the
                          concatenated String "Hello, world! At time: " + str(ticker)
  * ChROSManager       -> RegisterHandler / Initialize / Update(time) every step
Every "published" message is recorded (the framework keeps the latest payload),
so the published quantities are logged to CSV for verification rather than being
sent over a transport.

System type : ChSystemNSC (rigid impulsive contact).
Main bodies : fixed ground plate + one dynamic box (the tracked body).
Expected    : the box drops ~1 m, contacts the plate and settles; the custom
              handler emits a String "Hello, world! At time: <ticker>" each tick;
              clock/pose/twist/tf topics carry consistent values throughout.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / ROS framework) ===
TIME_STEP = 1.0e-3            # integrator step [s]
SIM_END = 6.0                # total simulated time [s]
RENDER_FPS = 30.0            # review-frame cadence [frames/s]

GROUND_SX, GROUND_SY, GROUND_SZ = 6.0, 6.0, 0.4   # ground plate full extents [m]
GROUND_TOP_Z = GROUND_SZ / 2.0                     # top surface z of the plate [m]

BOX_SX, BOX_SY, BOX_SZ = 0.5, 0.5, 0.5            # tracked box full extents [m]
BOX_DENSITY = 1000.0                               # box material density [kg/m^3]
BOX_DROP_HEIGHT = 1.0                              # clearance above plate top [m]
BOX_START_Z = GROUND_TOP_Z + BOX_SZ / 2.0 + BOX_DROP_HEIGHT  # spawn z [m]

FRICTION = 0.5               # contact friction coefficient
RESTITUTION = 0.3            # contact restitution (partial bounce)

# Custom-handler payload — the message prefix this handler owns and publishes.
CUSTOM_MESSAGE_PREFIX = "Hello, world! At time: "
CUSTOM_PUBLISH_RATE_HZ = 10.0   # custom-handler tick rate [Hz]

# Derived / precomputed-once values (never recomputed in the hot loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
CUSTOM_TICK_EVERY = max(1, round(1.0 / (CUSTOM_PUBLISH_RATE_HZ * TIME_STEP)))  # precomputed once

# Headless validation gate: fast, windowless physics check (see codegen_rules).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short check when validating


# === ROS-shaped handler framework (self-contained substitute for pychrono.ros) ===
# These mirror the pychrono.ros API shape. Each handler exposes Initialize() and
# Tick(time); the manager calls them via Update(). "Publishing" stores the latest
# message payload on the handler so it can be logged to CSV.
class ChROSHandler:
    """Base handler: fixed update rate, derived tick cadence (like pychrono.ros)."""

    def __init__(self, update_rate_hz):
        self.update_rate = float(update_rate_hz)
        # cache: tick cadence derived once from the rate, reused every Update
        self._tick_every = max(1, round(1.0 / (self.update_rate * TIME_STEP)))
        self.last_message = None

    def Initialize(self):
        """Allocate the publisher. No transport here — just mark ready."""
        self.last_message = None
        return True

    def Tick(self, time):
        raise NotImplementedError

    def Update(self, time, step_index):
        # Only tick on the handler's own cadence, matching ChROSHandler semantics.
        if step_index % self._tick_every == 0:
            self.Tick(time)


class ChROSClockHandler(ChROSHandler):
    """Publishes simulation time on /clock (rosgraph_msgs/Clock shape)."""

    def __init__(self, update_rate_hz=100.0):
        super().__init__(update_rate_hz)
        self.topic = "/clock"

    def Tick(self, time):
        self.last_message = {"clock_sec": time}


class ChROSBodyHandler(ChROSHandler):
    """Publishes a tracked body's pose (/pose) and twist (/twist)."""

    def __init__(self, update_rate_hz, body, topic_prefix):
        super().__init__(update_rate_hz)
        self.body = body  # cache: tracked body handle, reused every Tick
        self.pose_topic = topic_prefix + "/pose"
        self.twist_topic = topic_prefix + "/twist"
        self.last_pose = None
        self.last_twist = None

    def Tick(self, time):
        pos = self.body.GetPos()
        rot = self.body.GetRot()
        lin = self.body.GetPosDt()
        ang = self.body.GetAngVelParent()
        self.last_pose = (pos.x, pos.y, pos.z, rot.e0, rot.e1, rot.e2, rot.e3)
        self.last_twist = (lin.x, lin.y, lin.z, ang.x, ang.y, ang.z)
        self.last_message = {"pose": self.last_pose, "twist": self.last_twist}


class ChROSTFHandler(ChROSHandler):
    """Publishes a world -> body transform on /tf (tf2_msgs/TFMessage shape)."""

    def __init__(self, update_rate_hz, body, parent_frame, child_frame):
        super().__init__(update_rate_hz)
        self.body = body  # cache: tracked body handle, reused every Tick
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.topic = "/tf"

    def Tick(self, time):
        pos = self.body.GetPos()
        rot = self.body.GetRot()
        self.last_message = {
            "parent": self.parent_frame,
            "child": self.child_frame,
            "translation": (pos.x, pos.y, pos.z),
            "rotation": (rot.e0, rot.e1, rot.e2, rot.e3),
        }


class MyCustomHandler(ChROSHandler):
    """Custom handler publishing a String message on a user topic.

    Owns a `message` attribute equal to the prefix "Hello, world! At time: ".
    On each Tick it increments an integer ticker and publishes the concatenated
    String "Hello, world! At time: " + str(ticker).
    """

    def __init__(self, update_rate_hz, topic):
        super().__init__(update_rate_hz)
        self.topic = topic
        self.message = CUSTOM_MESSAGE_PREFIX  # the owned String prefix
        self.ticker = 0

    def Tick(self, time):
        # Publish a String: prefix concatenated with the current ticker value.
        payload = self.message + str(self.ticker)
        self.last_message = {"data": payload, "type": "String"}
        self.ticker += 1


class ChROSManager:
    """Registers handlers, initializes them, and updates them each step."""

    def __init__(self):
        self.handlers = []  # ordered list of registered handlers
        self._step_index = 0

    def RegisterHandler(self, handler):
        self.handlers.append(handler)

    def Initialize(self):
        # cache: resolve each Initialize once at startup
        for handler in self.handlers:
            handler.Initialize()
        return True

    def Update(self, time):
        for handler in self.handlers:
            handler.Update(time, self._step_index)
        self._step_index += 1


# === System & gravity === build the NSC world; gravity along -Z (Z-up world).
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # contact needed

# Shared NSC contact material for ground + box.
contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(FRICTION)
contact_mat.SetRestitution(RESTITUTION)

# === Bodies === fixed ground plate + one dynamic tracked box.
ground = chrono.ChBodyEasyBox(GROUND_SX, GROUND_SY, GROUND_SZ,
                              1000.0, True, True, contact_mat)  # visualize, collide
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

box = chrono.ChBodyEasyBox(BOX_SX, BOX_SY, BOX_SZ,
                           BOX_DENSITY, True, True, contact_mat)  # visualize, collide
box.SetPos(chrono.ChVector3d(0, 0, BOX_START_Z))
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(box)

# Sanity: the box must start clearly above the plate top, not interpenetrating.
assert BOX_START_Z - BOX_SZ / 2.0 > GROUND_TOP_Z, "box must spawn above the plate"

# === ROS manager & handlers === register clock, body pose/twist, tf, custom String.
ros_manager = ChROSManager()
clock_handler = ChROSClockHandler(update_rate_hz=100.0)
body_handler = ChROSBodyHandler(update_rate_hz=25.0, body=box, topic_prefix="/box")
tf_handler = ChROSTFHandler(update_rate_hz=25.0, body=box,
                            parent_frame="world", child_frame="box")
custom_handler = MyCustomHandler(update_rate_hz=CUSTOM_PUBLISH_RATE_HZ,
                                 topic="/chrono/custom")
ros_manager.RegisterHandler(clock_handler)
ros_manager.RegisterHandler(body_handler)
ros_manager.RegisterHandler(tf_handler)
ros_manager.RegisterHandler(custom_handler)
ros_manager.Initialize()

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ROS Handler Demo - String publisher on bouncing box")
    vis.Initialize()                                     # Initialize FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.0, -4.5, 2.5),
                  chrono.ChVector3d(0, 0, GROUND_TOP_Z))  # AFTER Initialize
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 24, 24,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_TOP_Z + 0.001),
                                   chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))

# === Output files / CSV writers === record published ROS quantities for verification.
os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
os.makedirs("cam", exist_ok=True)      # guard against missing motion-log dir

data_file = None
motion_file = None
data_writer = None
motion_writer = None
times, box_zs, box_vzs, tickers = [], [], [], []

try:
    # Guard the file opens specifically (disk / permission failures).
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:   # disk full / permission denied
        print("Failed to open output CSV:", exc)
        raise

    data_writer = csv.writer(data_file)
    data_writer.writerow([
        "time", "clock_sec",
        "box_x", "box_y", "box_z",
        "box_vx", "box_vy", "box_vz",
        "tf_child", "custom_ticker", "custom_message",
    ])
    motion_writer = csv.writer(motion_file)
    motion_writer.writerow([
        "time", "body", "x", "y", "z",
        "q0", "q1", "q2", "q3", "vx", "vy", "vz",
    ])

    # === Main loop === render-cadence outer loop; physics + ROS update inner batch.
    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile("frames/img_%06d.png" % frame)  # consecutive index
            frame += 1

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()

            # Advance the ROS framework: each handler ticks on its own cadence.
            ros_manager.Update(t)

            # Read back the latest published values for logging.
            clock_val = clock_handler.last_message["clock_sec"] if clock_handler.last_message else t
            pose = body_handler.last_pose
            twist = body_handler.last_twist
            child = tf_handler.child_frame
            cust = custom_handler.last_message
            cust_ticker = max(0, custom_handler.ticker - 1)
            cust_msg = cust["data"] if cust else ""

            if pose is not None and twist is not None:
                data_writer.writerow([
                    "%.4f" % t, "%.4f" % clock_val,
                    "%.5f" % pose[0], "%.5f" % pose[1], "%.5f" % pose[2],
                    "%.5f" % twist[0], "%.5f" % twist[1], "%.5f" % twist[2],
                    child, cust_ticker, cust_msg,
                ])
                motion_writer.writerow([
                    "%.4f" % t, "box",
                    "%.5f" % pose[0], "%.5f" % pose[1], "%.5f" % pose[2],
                    "%.6f" % pose[3], "%.6f" % pose[4], "%.6f" % pose[5], "%.6f" % pose[6],
                    "%.5f" % twist[0], "%.5f" % twist[1], "%.5f" % twist[2],
                ])
                times.append(t)
                box_zs.append(pose[2])
                box_vzs.append(twist[2])
                tickers.append(cust_ticker)

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= RUN_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close any open writers even if a step diverged mid-run.
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing === plot published box height/velocity and ticker vs time.
if times:
    t_arr = np.array(times)
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    axes[0].plot(t_arr, np.array(box_zs), color="tab:blue")
    axes[0].set_ylabel("box z [m]")
    axes[0].set_title("Published /box/pose height")
    axes[0].grid(True)

    axes[1].plot(t_arr, np.array(box_vzs), color="tab:green")
    axes[1].set_ylabel("box vz [m/s]")
    axes[1].set_title("Published /box/twist vertical velocity")
    axes[1].grid(True)

    axes[2].step(t_arr, np.array(tickers), color="tab:red", where="post")
    axes[2].set_ylabel("custom ticker")
    axes[2].set_xlabel("time [s]")
    axes[2].set_title('Published "%s<ticker>" String count' % CUSTOM_MESSAGE_PREFIX)
    axes[2].grid(True)

    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

print("Done. steps logged=%d, final custom ticker=%d, last message=%r"
      % (len(times), custom_handler.ticker, custom_handler.last_message))
