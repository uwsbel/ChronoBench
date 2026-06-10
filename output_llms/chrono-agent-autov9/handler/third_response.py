"""ROS-style publisher demo: a rigid box falls under gravity onto a fixed textured floor.

System type: ChSystemNSC (rigid, non-smooth contact) with Bullet collision.
Main bodies:
  - floor: a large fixed box with a concrete texture (the ground plane).
  - box: a dynamic cube with a checker/blue texture that drops from above,
    impacts the floor, and comes to rest.

ROS architecture note (substitution):
  This PyChrono 9.0.1 build ships NO `pychrono.ros` module. To honor the requested
  ROS publishing architecture, this script implements a SELF-CONTAINED,
  dependency-free publisher framework that mirrors the `pychrono.ros` SHAPE:
    * ChROSClockHandler  -> publishes the simulation /clock time,
    * ChROSBodyHandler   -> publishes the falling box's pose + twist,
    * ChROSTFHandler     -> publishes the world->box transform (TF),
    * ChROSContactHandler -> a custom handler publishing the floor contact count,
    * ChROSManager       -> RegisterHandler / Initialize / Update(time) each step.
  No network/middleware is used; "publishing" appends timestamped records that are
  flushed to CSV. The Chrono physics underneath is real.

Expected behavior:
  The box starts above the floor, accelerates downward at ~9.81 m/s^2, contacts
  the floor, and settles with its bottom resting on the floor top. All handlers
  publish at a fixed 10 Hz rate; published quantities are logged to CSV.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib

matplotlib.use("Agg")  # headless backend: render PNG without an X display
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / publishing) ===
TIME_STEP = 1.0e-3          # solver step [s]
SIM_END = 4.0               # total simulated time [s]
RENDER_FPS = 30.0           # review-video frame rate [frames/s]
GRAVITY_Z = -9.81           # gravitational acceleration [m/s^2]

FLOOR_SX = 8.0              # floor full extent along X [m]
FLOOR_SY = 8.0              # floor full extent along Y [m]
FLOOR_SZ = 0.4             # floor thickness along Z [m]
FLOOR_DENSITY = 1000.0     # floor density [kg/m^3] (fixed body, value irrelevant)
FLOOR_TOP_Z = 0.0          # world Z of the floor top surface [m]

BOX_SIZE = 0.6             # falling cube full edge length [m]
BOX_DENSITY = 500.0        # cube density [kg/m^3]
BOX_DROP_HEIGHT = 2.0      # initial clearance of cube bottom above floor top [m]

FRICTION = 0.6             # contact friction coefficient
RESTITUTION = 0.1          # contact restitution (slightly bouncy)

PUBLISH_RATE = 10.0        # ROS handler publish rate [Hz]

# Derived geometry / cadence (precomputed once) ---------------------------------
FLOOR_CENTER_Z = FLOOR_TOP_Z - FLOOR_SZ / 2.0          # precomputed once: floor box center Z
BOX_START_Z = FLOOR_TOP_Z + BOX_DROP_HEIGHT + BOX_SIZE / 2.0  # precomputed once: cube center Z at t=0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: physics steps per frame
PUBLISH_EVERY = max(1, round(1.0 / (PUBLISH_RATE * TIME_STEP)))  # precomputed once: steps per publish tick

# Headless validation gate: a short windowless physics check for fast validation.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))


# === ROS-shape publisher framework (self-contained substitution for pychrono.ros) ===
# These classes reproduce the pychrono.ros handler/manager SHAPE without any
# middleware. Each handler exposes Initialize()/Tick(time) and records the
# quantities it would publish on a ROS topic; ChROSManager drives them.
class ChROSHandler:
    """Base handler: fixed-rate ticking mirroring pychrono.ros handler cadence."""

    def __init__(self, update_rate, topic):
        self._update_rate = float(update_rate)   # publish rate [Hz]
        self._topic = topic                       # ROS topic name string
        self._period = 1.0 / float(update_rate)   # cache: publish period [s]
        self._next_pub = 0.0                       # next sim-time at which to publish
        self.records = []                          # accumulated published messages

    def Initialize(self):
        self._next_pub = 0.0
        return True

    def Tick(self, time):
        raise NotImplementedError

    def Update(self, time):
        # Publish only when the fixed-rate clock elapses (rate-limited like ROS).
        if time + 1e-9 >= self._next_pub:
            self.Tick(time)
            self._next_pub += self._period


class ChROSClockHandler(ChROSHandler):
    """Publishes the simulation clock on /clock (rosgraph_msgs/Clock shape)."""

    def __init__(self, update_rate, topic="/clock"):
        super().__init__(update_rate, topic)

    def Tick(self, time):
        self.records.append({"topic": self._topic, "t": time, "clock": time})


class ChROSBodyHandler(ChROSHandler):
    """Publishes a body's pose + twist (nav_msgs/Odometry shape)."""

    def __init__(self, update_rate, body, topic="/box/state"):
        super().__init__(update_rate, topic)
        self._body = body  # cache: tracked ChBody handle, reused every tick

    def Tick(self, time):
        p = self._body.GetPos()
        v = self._body.GetPosDt()
        q = self._body.GetRot()
        self.records.append({
            "topic": self._topic, "t": time,
            "px": p.x, "py": p.y, "pz": p.z,
            "vx": v.x, "vy": v.y, "vz": v.z,
            "qw": q.e0, "qx": q.e1, "qy": q.e2, "qz": q.e3,
        })


class ChROSTFHandler(ChROSHandler):
    """Publishes the world->body transform (tf2_msgs/TFMessage shape)."""

    def __init__(self, update_rate, body, parent="world", child="box", topic="/tf"):
        super().__init__(update_rate, topic)
        self._body = body  # cache: tracked ChBody handle, reused every tick
        self._parent = parent
        self._child = child

    def Tick(self, time):
        p = self._body.GetPos()
        q = self._body.GetRot()
        self.records.append({
            "topic": self._topic, "t": time,
            "parent": self._parent, "child": self._child,
            "tx": p.x, "ty": p.y, "tz": p.z,
            "qw": q.e0, "qx": q.e1, "qy": q.e2, "qz": q.e3,
        })


class ChROSContactHandler(ChROSHandler):
    """Custom handler: publishes the system contact count (std_msgs/Int32 shape)."""

    def __init__(self, update_rate, system, topic="/box/contacts"):
        super().__init__(update_rate, topic)
        self._system = system  # cache: ChSystem handle, reused every tick

    def Tick(self, time):
        self.records.append({
            "topic": self._topic, "t": time,
            "n_contacts": int(self._system.GetNumContacts()),
        })


class ChROSManager:
    """Mirrors pychrono.ros ChROSManager: RegisterHandler / Initialize / Update."""

    def __init__(self):
        self._handlers = []  # registered ChROSHandler instances

    def RegisterHandler(self, handler):
        self._handlers.append(handler)

    def Initialize(self):
        for h in self._handlers:
            h.Initialize()

    def Update(self, time):
        # Drive every registered handler once per physics step (each self-limits rate).
        for h in self._handlers:
            h.Update(time)

    @property
    def handlers(self):
        return self._handlers


def main():
    # === System & gravity === rigid-contact NSC system with Bullet collision.
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, GRAVITY_Z))
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetSolverType(chrono.ChSolver.Type_PSOR)
    sys.GetSolver().AsIterative().SetMaxIterations(120)

    # === Contact material === shared NSC material for floor + box.
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(FRICTION)
    mat.SetRestitution(RESTITUTION)

    # === Bodies === fixed textured floor + dynamic textured falling cube.
    floor = chrono.ChBodyEasyBox(FLOOR_SX, FLOOR_SY, FLOOR_SZ, FLOOR_DENSITY,
                                 True, True, mat)  # visualize, collide
    floor.SetPos(chrono.ChVector3d(0.0, 0.0, FLOOR_CENTER_Z))
    floor.SetFixed(True)
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)

    box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, BOX_DENSITY,
                               True, True, mat)  # visualize, collide
    box.SetPos(chrono.ChVector3d(0.0, 0.0, BOX_START_Z))
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    sys.Add(box)

    # Sanity: the cube must START above the floor (no initial interpenetration).
    assert BOX_START_Z - BOX_SIZE / 2.0 > FLOOR_TOP_Z, "cube must start above the floor top"

    # === ROS handlers === register clock + body + TF + contact handlers (10 Hz).
    ros_manager = ChROSManager()
    clock_handler = ChROSClockHandler(PUBLISH_RATE)
    body_handler = ChROSBodyHandler(PUBLISH_RATE, box, "/box/state")
    tf_handler = ChROSTFHandler(PUBLISH_RATE, box, "world", "box", "/tf")
    contact_handler = ChROSContactHandler(PUBLISH_RATE, sys, "/box/contacts")
    ros_manager.RegisterHandler(clock_handler)
    ros_manager.RegisterHandler(body_handler)
    ros_manager.RegisterHandler(tf_handler)
    ros_manager.RegisterHandler(contact_handler)
    ros_manager.Initialize()

    # === Visualization === full Irrlicht scene (window + sky + camera + lights + grid).
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("ROS handler demo - falling box on textured floor")
        vis.Initialize()                                    # Initialize FIRST (Irrlicht order)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(5.0, -6.0, 3.5),
                      chrono.ChVector3d(0.0, 0.0, 0.5))     # eye, target (AFTER Initialize)
        vis.AddTypicalLights()
        vis.AddGrid(0.5, 0.5, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, FLOOR_TOP_Z), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    # === Output dirs === guard against missing output directories.
    os.makedirs("frames", exist_ok=True)   # review PNG frames -> mp4 later
    os.makedirs("cam", exist_ok=True)       # motion log lives here

    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

    # Cache hot-loop handles once (avoid repeated getter calls each step).
    box_body = box       # cache: falling-cube handle, reused every step
    sim_clock = sys      # cache: system handle for GetChTime, reused every step

    # === Main loop === render-cadence outer loop; physics + logging in inner batch.
    data_file = None
    motion_file = None
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow([
            "time", "box_z", "box_vz", "box_pz_vx", "n_contacts", "published",
        ])
        motion_writer.writerow([
            "time", "body", "px", "py", "pz", "vx", "vy", "vz",
            "qw", "qx", "qy", "qz",
        ])

        frame = 0
        step_idx = 0
        while (HEADLESS or vis.Run()) and sim_clock.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index for ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                t = sim_clock.GetChTime()

                # Drive the ROS publisher framework (each handler self rate-limits).
                ros_manager.Update(t)
                published = 1 if (step_idx % PUBLISH_EVERY == 0) else 0

                # Per-step physics logging.
                p = box_body.GetPos()
                v = box_body.GetPosDt()
                q = box_body.GetRot()
                data_writer.writerow([
                    f"{t:.5f}", f"{p.z:.6f}", f"{v.z:.6f}", f"{v.x:.6f}",
                    sys.GetNumContacts(), published,
                ])
                motion_writer.writerow([
                    f"{t:.5f}", "box", f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
                    f"{v.x:.6f}", f"{v.y:.6f}", f"{v.z:.6f}",
                    f"{q.e0:.6f}", f"{q.e1:.6f}", f"{q.e2:.6f}", f"{q.e3:.6f}",
                ])

                sim_clock.DoStepDynamics(TIME_STEP)
                step_idx += 1
                if sim_clock.GetChTime() >= run_end:
                    break
    except (OSError, IOError) as exc:            # disk / permission failure on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:    # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close writers even if a step diverges mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === dump published ROS records + plot time series.
    try:
        with open("ros_published.csv", "w", newline="") as pub_file:
            pub_writer = csv.writer(pub_file)
            pub_writer.writerow(["topic", "time", "field", "value"])
            for h in ros_manager.handlers:
                for rec in h.records:
                    topic = rec["topic"]
                    t = rec["t"]
                    for k, val in rec.items():
                        if k in ("topic", "t"):
                            continue
                        pub_writer.writerow([topic, f"{t:.5f}", k, val])
    except (OSError, IOError) as exc:            # disk / permission failure on published-data CSV
        import traceback
        traceback.print_exc()
        raise

    # Plot the box drop trajectory + velocity from the logged CSV.
    try:
        data = np.genfromtxt("simulation_data.csv", delimiter=",", names=True)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
        ax1.plot(data["time"], data["box_z"], color="tab:blue", label="box z")
        ax1.axhline(FLOOR_TOP_Z + BOX_SIZE / 2.0, color="tab:red", ls="--",
                    label="rest height")
        ax1.set_ylabel("box center Z [m]")
        ax1.legend(); ax1.grid(True)
        ax2.plot(data["time"], data["box_vz"], color="tab:green", label="box vz")
        ax2.set_xlabel("time [s]"); ax2.set_ylabel("box vz [m/s]")
        ax2.legend(); ax2.grid(True)
        fig.suptitle("Falling box (ROS-published state) vs time")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)
    except (OSError, ValueError) as exc:         # missing CSV / malformed numeric data
        import traceback
        traceback.print_exc()
        raise

    # Report summary for the validation log.
    n_pub = sum(len(h.records) for h in ros_manager.handlers)
    print(f"[done] sim_end={run_end:.2f}s steps={step_idx} frames={'(headless)' if HEADLESS else 'written'} "
          f"published_msgs={n_pub} final_box_z={box.GetPos().z:.4f}")


if __name__ == "__main__":
    main()
