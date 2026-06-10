"""ROS-integrated rigid-body simulation: a box dropping onto a fixed floor.

Models a Chrono multi-body system (NSC contact, Z-up gravity) containing a fixed
floor and a single dynamic box that falls under gravity, contacts the floor, and
settles. A lightweight, self-contained ROS communication layer is reconstructed in
plain Python (this PyChrono build ships no ROS bindings): a handler base class, a
custom integer-publishing handler, clock/body/transform handlers, and a manager
that registers all handlers and ticks them at a fixed update rate each timestep.

System type:  ChSystemNSC (non-smooth rigid contact).
Main bodies:  fixed floor slab; dynamic falling box.
Expected behavior: the box falls ~from its spawn height, makes contact with the
floor, and comes to rest on top of it; meanwhile the ROS manager publishes the
clock, the box pose/transform, and a monotonically increasing integer message on
its topic at each ticked update.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === geometry / physics / ROS configuration (no bare literals downstream)
TIME_STEP = 2e-3            # integration step [s]
SIM_END = 6.0              # simulation duration [s]
RENDER_FPS = 30.0          # review render cadence [frames/s]
GRAVITY = -9.81            # gravitational acceleration along world Z [m/s^2]

FLOOR_SX, FLOOR_SY, FLOOR_SZ = 6.0, 6.0, 0.4   # floor full extents [m]
FLOOR_Z = 0.0                                  # floor center height [m]
FLOOR_DENSITY = 2000.0                         # floor material density [kg/m^3]

BOX_SX, BOX_SY, BOX_SZ = 0.8, 0.8, 0.8         # box full extents [m]
BOX_DENSITY = 500.0                            # box material density [kg/m^3]
DROP_HEIGHT = 2.5                              # box drop clearance above the floor top [m]

FRICTION = 0.6             # contact friction coefficient
RESTITUTION = 0.1          # contact restitution coefficient

ROS_TOPIC = "/chrono/int_counter"   # topic the custom handler publishes to
ROS_UPDATE_RATE = 25.0              # handler tick rate [Hz]

# Derived placement (precomputed once): floor top and box spawn center.
FLOOR_TOP_Z = FLOOR_Z + FLOOR_SZ / 2.0                  # precomputed once
BOX_SPAWN_Z = FLOOR_TOP_Z + DROP_HEIGHT + BOX_SZ / 2.0  # precomputed once
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === ROS layer (reconstructed in plain Python — this build has no ROS bindings) ===
# Mirrors the Chrono ROS API surface: a ChROSHandler base whose Tick(time) is rate
# limited, concrete clock/body/transform handlers, a custom integer publisher, and a
# manager that registers handlers and ticks them each step. No external ROS dependency.
class ChROSHandler:
    """Base handler ticked at a fixed update rate; subclasses override Tick()."""

    def __init__(self, update_rate):
        self.update_rate = float(update_rate)              # ticks per second [Hz]
        self._period = 1.0 / self.update_rate if self.update_rate > 0 else 0.0
        self._last_tick = -math.inf

    def Initialize(self):
        """Hook for one-time setup (advertise topics, etc.)."""
        return True

    def Update(self, time):
        """Rate gate: invoke Tick() only when a full period has elapsed."""
        if time - self._last_tick >= self._period:
            self._last_tick = time
            self.Tick(time)

    def Tick(self, time):
        """Publish this handler's payload at `time`. Overridden by subclasses."""
        raise NotImplementedError


class ChROSClockHandler(ChROSHandler):
    """Publishes the simulation clock onto /clock."""

    def __init__(self, update_rate=100.0):
        super().__init__(update_rate)
        self.last_clock = 0.0

    def Tick(self, time):
        self.last_clock = time   # /clock <- sim time


class ChROSBodyHandler(ChROSHandler):
    """Publishes a body's pose (position + orientation) onto a per-body topic."""

    def __init__(self, body, topic, update_rate=25.0):
        super().__init__(update_rate)
        self.body = body                  # cache: body handle reused every tick
        self.topic = topic
        self.last_pose = None

    def Tick(self, time):
        pos = self.body.GetPos()
        rot = self.body.GetRot()
        self.last_pose = (pos.x, pos.y, pos.z, rot.e0, rot.e1, rot.e2, rot.e3)


class ChROSTFHandler(ChROSHandler):
    """Publishes a transform (parent->child frame) onto /tf for the tracked body."""

    def __init__(self, body, frame_id, child_frame_id, update_rate=25.0):
        super().__init__(update_rate)
        self.body = body                  # cache: body handle reused every tick
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.last_transform = None

    def Tick(self, time):
        p = self.body.GetPos()
        self.last_transform = (self.frame_id, self.child_frame_id, p.x, p.y, p.z)


class ChROSIntPublisherHandler(ChROSHandler):
    """Custom handler: publishes a monotonically increasing integer on `topic`."""

    def __init__(self, topic, update_rate=ROS_UPDATE_RATE):
        super().__init__(update_rate)
        self.topic = topic
        self.counter = 0
        self.last_published = None

    def Initialize(self):
        self.counter = 0   # reset publish counter at advertise time
        return True

    def Tick(self, time):
        msg = self.counter          # std_msgs/Int32-style integer payload
        self.last_published = msg
        self.counter += 1


class ChROSPythonManager:
    """Owns the registered handlers and ticks them once per simulation step."""

    def __init__(self):
        self.handlers = []

    def RegisterHandler(self, handler):
        self.handlers.append(handler)

    def Initialize(self):
        for h in self.handlers:
            h.Initialize()

    def Update(self, time, time_step):
        """Advance ROS communication: rate-gated Update() on every handler."""
        for h in self.handlers:
            h.Update(time)


# === System & gravity === NSC rigid-contact world, Z-up gravity
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY))
# Scene has contact (box falls onto floor) -> Bullet collision system is REQUIRED.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)

# === Contact material === shared NSC material for floor and box
contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(FRICTION)
contact_mat.SetRestitution(RESTITUTION)

# === Bodies === fixed floor slab + dynamic falling box
floor = chrono.ChBodyEasyBox(FLOOR_SX, FLOOR_SY, FLOOR_SZ, FLOOR_DENSITY, True, True, contact_mat)  # visualize, collide
floor.SetPos(chrono.ChVector3d(0, 0, FLOOR_Z))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

box = chrono.ChBodyEasyBox(BOX_SX, BOX_SY, BOX_SZ, BOX_DENSITY, True, True, contact_mat)  # visualize, collide
box.SetPos(chrono.ChVector3d(0, 0, BOX_SPAWN_Z))
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(box)

# === ROS manager === clock + body + transform + custom integer handlers
ros_manager = ChROSPythonManager()
ros_manager.RegisterHandler(ChROSClockHandler(update_rate=100.0))
ros_manager.RegisterHandler(ChROSBodyHandler(box, "/chrono/box/pose", update_rate=ROS_UPDATE_RATE))
ros_manager.RegisterHandler(ChROSTFHandler(box, "world", "box", update_rate=ROS_UPDATE_RATE))
int_handler = ChROSIntPublisherHandler(ROS_TOPIC, update_rate=ROS_UPDATE_RATE)  # cache: reused for logging
ros_manager.RegisterHandler(int_handler)
ros_manager.Initialize()

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ROS-integrated box on floor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, -6, 4), chrono.ChVector3d(0, 0, FLOOR_TOP_Z))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 24, 24,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, FLOOR_TOP_Z + 0.001), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === advance physics + tick ROS each step; render at cadence


try:
    frame = 0
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            ros_manager.Update(t, TIME_STEP)
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
