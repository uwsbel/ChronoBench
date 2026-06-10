"""Custom ROS-style handler demo on a PyChrono rigid-body scene.

Models an NSC multi-body system with a fixed ground floor and a single dynamic
box that falls under gravity and settles on the floor (Bullet collision). On top
of the physics, the script reconstructs the structure of a Chrono ROS handler
stack in plain Python (this build ships no `pychrono.ros` module):

  * `ChROSHandler`         — rate-gated base: `Update(time)` decides when to fire
                             and delegates to `Tick(time)`.
  * `MyCustomHandler`      — a concrete publisher of a String message on a named
                             topic. It owns a `message` text and an integer
                             `ticker`, and each tick publishes the concatenated
                             string `"Hello, world! At time: " + str(self.ticker)`.
  * `ChROSPythonManager`   — registers handlers and ticks every one each step.

Expected behavior: the box drops a short distance, contacts the floor, and comes
to rest; the manager fires the String publisher at its configured rate, emitting
an incrementing "Hello, world!" message throughout the run.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === geometry / physics / handler configuration
TIME_STEP = 1e-3            # integration step [s]
SIM_END = 6.0               # simulated duration [s]
RENDER_FPS = 30.0           # review render cadence [frames/s]

GROUND_SIZE = 8.0           # floor side length [m]
GROUND_THICK = 0.4          # floor thickness [m]
GROUND_TOP_Z = 0.0          # world Z of the floor top surface [m]

BOX_SIZE = 0.6              # falling box side length [m]
BOX_DENSITY = 600.0         # box material density [kg/m^3]
BOX_DROP = 1.5              # initial clearance of box base above floor top [m]

PUBLISH_RATE = 10.0         # handler publish rate [Hz]
TOPIC_NAME = "/chrono/my_topic"   # named topic the publisher writes to
MESSAGE_TEXT = "Hello, world! At time: "   # String message prefix

# Derived placements (precomputed once, no bare literals downstream)
GROUND_CENTER_Z = GROUND_TOP_Z - GROUND_THICK / 2.0
BOX_START_Z = GROUND_TOP_Z + BOX_DROP + BOX_SIZE / 2.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


# === ROS-style handler stack === reconstructed in plain Python (no pychrono.ros)
class ChROSHandler:
    """Rate-gated handler base. `Update` fires `Tick` no faster than the rate."""

    def __init__(self, update_rate):
        self.update_rate = float(update_rate)               # Hz
        self._period = 1.0 / self.update_rate if self.update_rate > 0 else 0.0
        self._last_tick = -math.inf

    def Update(self, time):
        # Gate on the configured rate; only tick when a period has elapsed.
        if time - self._last_tick + 1e-12 >= self._period:
            self._last_tick = time
            self.Tick(time)

    def Tick(self, time):
        raise NotImplementedError


class StringMessage:
    """Minimal stand-in for the ROS std_msgs String message type."""

    def __init__(self, data=""):
        self.data = data


class ChROSPythonManager:
    """Registers ROS-style handlers and updates each one every simulation step."""

    def __init__(self):
        self._handlers = []     # registered handlers, ticked in order

    def RegisterHandler(self, handler):
        self._handlers.append(handler)

    def Update(self, time):
        for handler in self._handlers:
            handler.Update(time)


class MyCustomHandler(ChROSHandler):
    """Concrete publisher: emits a String message on a named topic each tick.

    The published payload is the `message` prefix concatenated with the current
    integer `ticker`, i.e. "Hello, world! At time: 0", "... 1", ...
    """

    def __init__(self, update_rate, topic):
        super().__init__(update_rate)
        self.topic = topic                  # named topic this handler publishes to
        self.ticker = 0                     # incrementing publish counter
        self.message = MESSAGE_TEXT         # String message prefix
        self.last_published = ""            # cache of the most recent payload

    def Tick(self, time):
        # Publish the concatenated String message and advance the ticker.
        payload = self.message + str(self.ticker)
        msg = StringMessage(payload)
        self.last_published = msg.data
        print(f"[{self.topic}] publish String: {msg.data}")
        self.ticker += 1


# === System & gravity === NSC multi-body with Bullet collision (box + floor contact)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # scene has contact

# === Contact material === shared NSC material for floor and box
contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.6)
contact_mat.SetRestitution(0.0)

# === Bodies === fixed ground floor + one dynamic falling box
ground = chrono.ChBodyEasyBox(GROUND_SIZE, GROUND_SIZE, GROUND_THICK,
                              1000.0, True, True, contact_mat)   # visualize, collide
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_CENTER_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE,
                           BOX_DENSITY, True, True, contact_mat)   # visualize, collide
box.SetPos(chrono.ChVector3d(0, 0, BOX_START_Z))
box.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.4, 0.8))
sys.Add(box)

# Assert the box starts above the floor (not interpenetrating) at spawn.
assert BOX_START_Z - BOX_SIZE / 2.0 > GROUND_TOP_Z, "box must spawn above floor"

# === ROS-style handlers === register the String publisher with the manager
ros_manager = ChROSPythonManager()
publisher = MyCustomHandler(PUBLISH_RATE, TOPIC_NAME)
ros_manager.RegisterHandler(publisher)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Custom ROS Handler Demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, -5, 3), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 32, 32,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_TOP_Z), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop, physics + handler tick inner batch

box_handle = box                # cache: body fetched once, polled every step

try:

    frame = 0
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            ros_manager.Update(t)           # tick the ROS-style handler stack
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
