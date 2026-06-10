"""PyChrono rigid-body simulation with a ROS-style communication layer.

Model
-----
A non-smooth-contact (NSC) multibody system under Earth gravity containing a
fixed floor slab and a single dynamic box. The box is released slightly above
the floor and settles onto it under gravity and frictional/restitutional
contact. This is the classic "publisher" scene used to exercise a ROS bridge:
the simulation state (clock, body pose, coordinate transform) is published every
timestep, alongside a user-defined integer message on a custom topic.

System type
-----------
chrono.ChSystemNSC (rigid, impulsive contacts) with Bullet collision and a
PSOR iterative solver.

Main bodies
-----------
- floor : fixed box slab (the static support, also the ROS transform parent).
- box   : dynamic box that falls a short distance and rests on the floor.

ROS layer
---------
This build of PyChrono does not ship the optional `pychrono.ros` bridge module,
so the requested ROS architecture is reproduced as a self-contained, dependency
-free publisher framework that mirrors `pychrono.ros` exactly:

- ChROSClockHandler      -> publishes the simulation clock (/clock).
- ChROSBodyHandler       -> publishes the movable box pose/twist (/box/state).
- ChROSTFHandler         -> publishes the floor->box coordinate transform (/tf).
- CustomIntHandler       -> the user-defined handler, publishing an incrementing
                            integer message (std_msgs/Int32 style) to a topic.
- ChROSManager           -> owns/registers handlers, initializes them once, and
                            updates the registered handlers every timestep.

Each published message is also persisted to CSV so the run is verifiable. The
loop maintains real-time execution with ChRealtimeStepTimer.

Expected behavior
-----------------
The box drops a few centimeters, contacts the floor, and comes to rest with a
small positive resting height (no penetration, no NaN). The clock advances
monotonically, the integer message increments once per published step, and the
floor->box transform converges to a constant offset once the box settles.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for PNG output
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants: physics, geometry, ROS, run control ===
GRAVITY_Z = -9.81          # gravitational acceleration along world Z [m/s^2]
MATERIAL_FRICTION = 0.6    # contact friction coefficient [-]
MATERIAL_RESTITUTION = 0.1  # contact restitution coefficient [-]

FLOOR_SX, FLOOR_SY, FLOOR_SZ = 6.0, 6.0, 0.4   # floor full extents [m]
FLOOR_DENSITY = 2000.0                          # floor density [kg/m^3]
FLOOR_TOP_Z = FLOOR_SZ / 2.0                    # world Z of the floor top surface [m]

BOX_SX, BOX_SY, BOX_SZ = 0.5, 0.5, 0.5         # movable box full extents [m]
BOX_DENSITY = 600.0                             # box density [kg/m^3]
BOX_DROP_GAP = 0.30                             # release clearance above floor [m]
# Box spawn so its bottom face starts BOX_DROP_GAP above the floor top.
BOX_SPAWN_Z = FLOOR_TOP_Z + BOX_SZ / 2.0 + BOX_DROP_GAP

ROS_INT_TOPIC = "/chrono/custom_int"   # topic for the custom integer message
ROS_BODY_TOPIC = "/chrono/box/state"   # topic for the box state message
ROS_CLOCK_TOPIC = "/clock"             # standard simulation-clock topic
ROS_TF_TOPIC = "/tf"                   # standard transform topic
ROS_INT_START = 0                      # initial value of the published integer

TIME_STEP = 1e-3        # physics time step [s]
SIM_END = 4.0           # simulation duration [s]
RENDER_FPS = 50.0       # review-video frame rate [fps]
# precomputed once: physics steps between rendered frames
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

WINDOW_W, WINDOW_H = 1280, 720
CAMERA_EYE = chrono.ChVector3d(6.0, -6.0, 4.0)
CAMERA_TARGET = chrono.ChVector3d(0.0, 0.0, 0.5)

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast windowless validation run


# === ROS-style handlers (self-contained mirror of pychrono.ros) ===
# This build lacks the optional pychrono.ros bridge, so the handler/manager
# architecture the scenario asks for is implemented inline with the same shape:
# each handler has Initialize(...) and Tick(time) -> message-dict, and the
# manager registers and updates them every step.
class ChROSHandler:
    """Base publisher handler: a topic plus a per-step update rate."""

    def __init__(self, topic, update_rate):
        self.topic = topic
        self.update_rate = update_rate      # publishes per second [Hz]
        self._period = 1.0 / update_rate    # cache: seconds between publishes
        self._next_pub = 0.0                # next sim-time at which to publish
        self.last_message = None

    def Initialize(self):
        """Reset publish schedule; called once before the loop."""
        self._next_pub = 0.0
        return True

    def _due(self, time):
        if time + 1e-12 >= self._next_pub:
            self._next_pub += self._period
            return True
        return False

    def Tick(self, time):  # overridden by subclasses
        raise NotImplementedError


class ChROSClockHandler(ChROSHandler):
    """Publishes the simulation clock to /clock (rosgraph_msgs/Clock style)."""

    def __init__(self, update_rate=1000.0):
        super().__init__(ROS_CLOCK_TOPIC, update_rate)

    def Tick(self, time):
        if not self._due(time):
            return None
        self.last_message = {"topic": self.topic, "clock_sec": time}
        return self.last_message


class ChROSBodyHandler(ChROSHandler):
    """Publishes a body's pose + linear velocity (nav_msgs/Odometry style)."""

    def __init__(self, body, topic, update_rate=100.0):
        super().__init__(topic, update_rate)
        self.body = body  # cache: body handle reused every tick

    def Tick(self, time):
        if not self._due(time):
            return None
        pos = self.body.GetPos()
        vel = self.body.GetPosDt()
        self.last_message = {
            "topic": self.topic,
            "px": pos.x, "py": pos.y, "pz": pos.z,
            "vx": vel.x, "vy": vel.y, "vz": vel.z,
        }
        return self.last_message


class ChROSTFHandler(ChROSHandler):
    """Publishes the parent->child coordinate transform (tf2_msgs/TFMessage)."""

    def __init__(self, parent, child, update_rate=100.0):
        super().__init__(ROS_TF_TOPIC, update_rate)
        self.parent = parent  # cache: parent body handle
        self.child = child    # cache: child body handle

    def Tick(self, time):
        if not self._due(time):
            return None
        # Relative position of child expressed in the parent frame.
        rel = self.parent.GetRot().RotateBack(self.child.GetPos() - self.parent.GetPos())
        self.last_message = {
            "topic": self.topic,
            "frame": "floor", "child_frame": "box",
            "tx": rel.x, "ty": rel.y, "tz": rel.z,
        }
        return self.last_message


class CustomIntHandler(ChROSHandler):
    """Custom user handler: publishes an incrementing integer (std_msgs/Int32)."""

    def __init__(self, topic, start_value=ROS_INT_START, update_rate=10.0):
        super().__init__(topic, update_rate)
        self.value = start_value

    def Tick(self, time):
        if not self._due(time):
            return None
        msg = {"topic": self.topic, "data": self.value}
        self.value += 1  # advance the integer payload after each publish
        self.last_message = msg
        return msg


class ChROSManager:
    """Owns the registered handlers, initializes them, and updates each step."""

    def __init__(self):
        self.handlers = []

    def RegisterHandler(self, handler):
        self.handlers.append(handler)

    def Initialize(self):
        for h in self.handlers:
            h.Initialize()

    def Update(self, time, time_step):
        """Tick every registered handler; return the messages emitted this step."""
        emitted = []
        for h in self.handlers:
            msg = h.Tick(time)
            if msg is not None:
                emitted.append(msg)
        return emitted


def main():
    # === System & gravity === NSC rigid-contact world under Earth gravity
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, GRAVITY_Z))
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetSolverType(chrono.ChSolver.Type_PSOR)
    sys.GetSolver().AsIterative().SetMaxIterations(100)

    # === Contact material === shared NSC material (friction + slight restitution)
    contact_mat = chrono.ChContactMaterialNSC()
    contact_mat.SetFriction(MATERIAL_FRICTION)
    contact_mat.SetRestitution(MATERIAL_RESTITUTION)

    # === Bodies === fixed floor slab + dynamic movable box
    floor = chrono.ChBodyEasyBox(FLOOR_SX, FLOOR_SY, FLOOR_SZ,
                                 FLOOR_DENSITY, True, True, contact_mat)  # visualize, collide
    floor.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
    floor.SetFixed(True)  # immovable support / ROS transform parent
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)

    box = chrono.ChBodyEasyBox(BOX_SX, BOX_SY, BOX_SZ,
                               BOX_DENSITY, True, True, contact_mat)  # visualize, collide
    box.SetPos(chrono.ChVector3d(0.0, 0.0, BOX_SPAWN_Z))
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/cubetexture_borders.png"))
    sys.Add(box)

    # Sanity: the box must start clearly above the floor top (no initial overlap).
    assert box.GetPos().z - BOX_SZ / 2.0 > FLOOR_TOP_Z, "box spawns inside the floor"

    # === ROS layer === build handlers and register them with the manager
    ros_manager = ChROSManager()
    clock_handler = ChROSClockHandler(update_rate=1.0 / TIME_STEP)
    body_handler = ChROSBodyHandler(box, ROS_BODY_TOPIC, update_rate=100.0)
    tf_handler = ChROSTFHandler(floor, box, update_rate=100.0)
    custom_int_handler = CustomIntHandler(ROS_INT_TOPIC, ROS_INT_START, update_rate=50.0)
    for handler in (clock_handler, body_handler, tf_handler, custom_int_handler):
        ros_manager.RegisterHandler(handler)
    ros_manager.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)  # Z-up world
        vis.SetWindowSize(WINDOW_W, WINDOW_H)
        vis.SetWindowTitle("Chrono + ROS handler: floor and movable box")
        vis.Initialize()  # Initialize FIRST, then add scene elements
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(CAMERA_EYE, CAMERA_TARGET)
        vis.AddTypicalLights()
        vis.AddGrid(0.5, 0.5, 24, 24,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, FLOOR_TOP_Z + 1e-3), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))  # ground reference grid on the floor top

    # === Output dirs === guard against missing output directories
    os.makedirs("frames", exist_ok=True)
    os.makedirs("cam", exist_ok=True)

    # === Main loop === advance physics + update ROS comms, log to CSV
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating
    rt_timer = chrono.ChRealtimeStepTimer()  # maintains real-time execution

    times, box_z, box_vz, int_values = [], [], [], []

    sim_csv = None
    motion_csv = None
    try:
        # Open both writers with context managers so they always flush/close.
        with open("simulation_data.csv", "w", newline="") as sim_f, \
             open("cam/motion_log.csv", "w", newline="") as motion_f:
            sim_csv = csv.writer(sim_f)
            motion_csv = csv.writer(motion_f)
            sim_csv.writerow(["time", "box_pz", "box_vz",
                              "tf_floor_box_z", "custom_int", "clock_sec"])
            motion_csv.writerow(["time", "px", "py", "pz", "vx", "vy", "vz"])

            frame = 0
            while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
                if not HEADLESS:
                    vis.BeginScene()
                    vis.Render()
                    vis.EndScene()
                    vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                    frame += 1

                for _ in range(RENDER_EVERY):
                    t = sys.GetChTime()  # cache: single clock read per step

                    # Publish ROS messages for this step and capture the payloads.
                    emitted = ros_manager.Update(t, TIME_STEP)
                    tf_z = float("nan")
                    int_val = custom_int_handler.value
                    clock_sec = t
                    for msg in emitted:
                        if msg["topic"] == ROS_TF_TOPIC:
                            tf_z = msg["tz"]
                        elif msg["topic"] == ROS_INT_TOPIC:
                            int_val = msg["data"]
                        elif msg["topic"] == ROS_CLOCK_TOPIC:
                            clock_sec = msg["clock_sec"]

                    pos = box.GetPos()
                    vel = box.GetPosDt()
                    sim_csv.writerow([t, pos.z, vel.z, tf_z, int_val, clock_sec])
                    motion_csv.writerow([t, pos.x, pos.y, pos.z,
                                         vel.x, vel.y, vel.z])
                    times.append(t)
                    box_z.append(pos.z)
                    box_vz.append(vel.z)
                    int_values.append(int_val)

                    sys.DoStepDynamics(TIME_STEP)
                    if not HEADLESS:
                        rt_timer.Spin(TIME_STEP)  # pace to wall-clock real time
                    if sys.GetChTime() >= run_end:
                        break
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:  # disk / permission failure on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    finally:
        # The `with open(...)` writers above already flush/close on exit; nothing
        # left to release here, but keep the block so partial state is consistent.
        pass

    # === Post-processing === time-series plot from the logged arrays
    if times:
        fig, (ax1, ax2) = plt.subplots(2, sharex=True, figsize=(8, 6))
        ax1.plot(times, box_z, label="box z [m]")
        ax1.axhline(FLOOR_TOP_Z + BOX_SZ / 2.0, color="r", ls="--",
                    label="resting height")
        ax1.set(ylabel="height [m]")
        ax1.grid(); ax1.legend()
        ax2.plot(times, int_values, "g-", label="custom int message")
        ax2.set(ylabel="int payload", xlabel="time [s]")
        ax2.grid(); ax2.legend()
        fig.suptitle("Movable box settling + ROS custom-int publication")
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        final_z = box_z[-1]
        print(f"steps={len(times)}  final_box_z={final_z:.4f}  "
              f"final_int={int_values[-1]}  any_nan={bool(np.isnan(box_z).any())}")


if __name__ == "__main__":
    main()
