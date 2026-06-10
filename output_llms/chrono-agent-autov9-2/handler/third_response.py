"""Rigid box dropped onto a textured floor with ROS-style publishing handlers.

Models a single dynamic rigid box (NSC contact) falling under gravity onto a
fixed, textured floor slab inside an Irrlicht window. Alongside the physics, the
script defines a lightweight ROS-handler architecture in plain Python: a
rate-gated handler base class, a clock handler and a body-state handler that
"publish" the simulated time and the box pose/velocity at a fixed publish rate,
and a manager that registers the handlers and ticks them every step.

System type: ChSystemNSC (non-smooth rigid contact between box and floor).
Main bodies: a fixed floor box and a dynamic falling box.
Expected behavior: the box drops, contacts the floor, and comes to rest on it
while the handlers emit clock and body-state messages at the configured rate.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Simulation constants === geometry / physics / rate parameters (no bare literals downstream)
time_step = 1e-3              # integration step [s]
sim_end = 4.0                # total simulated time [s]
render_fps = 50.0            # review-render cadence [frames/s]

publish_rate = 10.0          # ROS handler publish rate [Hz]

floor_sx, floor_sy, floor_sz = 8.0, 8.0, 0.4       # floor slab full extents [m]
box_sx, box_sy, box_sz = 0.6, 0.6, 0.6             # falling box full extents [m]
box_density = 600.0                                # box material density [kg/m^3]
drop_height = 2.0                                  # box center start height above floor top [m]

floor_top_z = floor_sz / 2.0                       # world z of the floor's top face
box_start_z = floor_top_z + box_sz / 2.0 + drop_height   # box COM spawn height

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once: physics steps per frame


# === ROS handler architecture (reconstructed in plain Python) ===
# This build has no pychrono.ros module, so the ROS publishing structure is
# expressed directly: a rate-gated handler base whose Update(time) decides when
# to call the concrete Tick(time), and a manager that registers + ticks them.
class ChROSHandler:
    """Base publishing handler: gates Tick(time) to a fixed update rate [Hz]."""

    def __init__(self, update_rate, topic):
        self.update_rate = float(update_rate)      # publish frequency [Hz]
        self.topic = topic                         # logical ROS topic name
        self._period = 1.0 / float(update_rate)    # cache: publish interval [s]
        self._next_time = 0.0                      # next simulated time to publish at
        self.publish_count = 0                     # number of emitted messages

    def Update(self, time):
        """Rate gate: publish only once per 1/update_rate of simulated time."""
        if time + 1e-12 >= self._next_time:
            self.Tick(time)
            self.publish_count += 1
            self._next_time += self._period
            return True
        return False

    def Tick(self, time):
        """Emit one message. Concrete handlers override this."""
        raise NotImplementedError


class ChROSClockHandler(ChROSHandler):
    """Publishes the current simulated clock time on /clock."""

    def __init__(self, update_rate):
        super().__init__(update_rate, "/clock")
        self.last_clock = 0.0                      # most recently published sim time

    def Tick(self, time):
        self.last_clock = time                     # rosgraph_msgs/Clock payload


class ChROSBodyHandler(ChROSHandler):
    """Publishes a rigid body's pose + linear velocity on a state topic."""

    def __init__(self, update_rate, topic, body):
        super().__init__(update_rate, topic)
        self.body = body                           # cache: published body handle
        self.last_pose = (0.0, 0.0, 0.0)           # most recent published position
        self.last_vel = (0.0, 0.0, 0.0)            # most recent published lin. velocity

    def Tick(self, time):
        p = self.body.GetPos()                     # geometry_msgs/Pose position
        v = self.body.GetPosDt()                   # geometry_msgs/Twist linear part
        self.last_pose = (p.x, p.y, p.z)
        self.last_vel = (v.x, v.y, v.z)


class ChROSPythonManager:
    """Registers ROS handlers and updates each one every physics step."""

    def __init__(self):
        self.handlers = []                         # registered ChROSHandler list

    def RegisterHandler(self, handler):
        self.handlers.append(handler)

    def Update(self, time, step):
        """Tick every handler; each self-gates to its own publish rate."""
        for handler in self.handlers:
            handler.Update(time)


# === System & gravity === NSC rigid-contact world with standard gravity along -Z
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED: box/floor contact
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)

# === Contact material === shared NSC material for box and floor
contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.6)
contact_mat.SetRestitution(0.0)

# === Bodies === fixed textured floor + dynamic textured falling box
floor = chrono.ChBodyEasyBox(floor_sx, floor_sy, floor_sz, 1000.0, True, True, contact_mat)  # visualize, collide
floor.SetPos(chrono.ChVector3d(0, 0, 0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

box = chrono.ChBodyEasyBox(box_sx, box_sy, box_sz, box_density, True, True, contact_mat)  # visualize, collide
box.SetPos(chrono.ChVector3d(0, 0, box_start_z))
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/cubetexture_bluewhite.png"))
sys.Add(box)

# === ROS handlers === clock + box-state publishers ticked through the manager at publish_rate
ros_manager = ChROSPythonManager()
clock_handler = ChROSClockHandler(publish_rate)
box_handler = ChROSBodyHandler(publish_rate, "/box/state", box)
ros_manager.RegisterHandler(clock_handler)
ros_manager.RegisterHandler(box_handler)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Box on floor with ROS publishing handlers")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, -6, 4), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, floor_top_z + 0.001), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render at fixed cadence; step physics + tick ROS handlers in the inner batch


step_number = 0
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            ros_manager.Update(t, step_number)
            sys.DoStepDynamics(time_step)
            step_number += 1
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    print(f"clock messages published: {clock_handler.publish_count}; "
          f"box messages published: {box_handler.publish_count}")

# === Post-processing === assemble the review video + timeseries plot, then clean frames
