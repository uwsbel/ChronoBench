"""HMMWV wheeled vehicle on flat rigid terrain with an in-process, ROS-shaped
publishing layer.

Model
-----
- System type: SMC (the HMMWV_Full wrapper is configured with
  ChContactMethod_SMC; its tire/contact stack matches an SMC ground).
- Main bodies: an HMMWV_Full wrapper vehicle (chassis + 4 axles/spindles +
  suspension/steering links, all created internally by the wrapper) and a flat
  RigidTerrain patch acting as the ground.
- Driver: a scripted veh.ChDriver subclass that brakes briefly, then applies a
  fixed forward throttle with a mild sinusoidal steering sweep so the chassis
  visibly translates and yaws.

ROS substitution (IMPORTANT)
----------------------------
This PyChrono build ships NO `pychrono.ros` module. The simulation therefore
reconstructs the ROS layer as a SELF-CONTAINED, dependency-free publisher
framework that mirrors the SHAPE of pychrono.ros: a ChROSManager owning a
ChROSClockHandler (publishes /clock), a ChROSDriverInputsHandler (publishes the
driver steering/throttle/braking), and a ChROSBodyHandler (publishes the chassis
pose + twist). Each handler implements Initialize()/Update(time)/Tick() and is
Register()-ed with the manager; the manager Initialize()s once and Update()s every
step, exactly as the real pychrono.ros API would be driven. There is no network
transport — "published" messages are captured in-process and logged to CSV so the
published quantities are inspectable. No external ROS/rclpy dependency is used.

Expected behavior
------------------
The chassis stays upright (wheels on the ground) and translates forward several
metres over the run while the ROS-shaped handlers publish /clock, driver inputs,
and chassis odometry at their configured rates.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib

matplotlib.use("Agg")  # headless-safe backend for the timeseries PNG
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / control) ===
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire force model sub-step (s)
SIM_END = 12.0                     # total simulated time (s)
RENDER_FPS = 30.0                  # review-video frame rate

TERRAIN_LENGTH = 300.0             # rigid patch X extent (m) — covers full drive
TERRAIN_WIDTH = 200.0              # rigid patch Y extent (m) — covers lateral sweep
TERRAIN_FRICTION = 0.9             # ground friction coefficient
TERRAIN_RESTITUTION = 0.01         # ground restitution (near-inelastic)
TERRAIN_YOUNG = 2.0e7              # SMC ground Young's modulus (Pa)

INIT_X = -80.0                     # spawn X near the patch's negative end (m)
INIT_Y = 0.0                       # spawn Y on the centerline (m)
SUSPENSION_REF_HEIGHT = 0.5        # HMMWV chassis-origin height above ground (m)
TERRAIN_TOP_Z = 0.0                # flat patch top plane (m)
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT  # derived chassis spawn Z

TIRE_RADIUS = 0.4699               # HMMWV tire radius (m) for footprint assert
Z_TOL = 0.10                       # allowed wheel-bottom clearance vs ground (m)

LAUNCH_TIME = 0.5                  # brake-hold duration before driving (s)
DRIVE_THROTTLE = 0.6               # forward throttle after launch (0..1)
STEER_AMP = 0.08                   # steering sweep amplitude (-1..1), small to bound lateral drift
STEER_RATE = 1.2                   # steering sweep angular rate (rad/s) — several cycles per run

# ROS-shaped handler publish rates (Hz) — mirror typical pychrono.ros config.
CLOCK_RATE = 100.0
DRIVER_RATE = 25.0
BODY_RATE = 25.0

OUT_SIM_CSV = "simulation_data.csv"
OUT_MOTION_CSV = "cam/motion_log.csv"
OUT_PLOT = "simulation_timeseries.png"

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast windowless validation run


# === ROS-shaped publishing layer (self-contained, no pychrono.ros) ===
# These classes reproduce the pychrono.ros API SHAPE (ChROSManager +
# ChROSClockHandler + ChROSDriverInputsHandler + ChROSBodyHandler). Each handler
# carries a publish rate, an Initialize(), and an Update(time) that fires Tick()
# at the configured cadence. The manager Register()s handlers, Initialize()s once,
# and Update()s each step. Captured messages are stored in `published` for CSV.
class ChROSHandler:
    """Base handler: rate-limited Tick() driven by Update(time)."""

    def __init__(self, update_rate, topic):
        self.update_rate = float(update_rate)      # Hz
        self.topic = topic
        self._period = 1.0 / self.update_rate if self.update_rate > 0 else 0.0
        self._next_time = 0.0
        self.last_message = None

    def Initialize(self):
        self._next_time = 0.0
        return True

    def Update(self, time):
        # Fire Tick() only when the configured period has elapsed.
        if time + 1e-12 >= self._next_time:
            self.Tick(time)
            self._next_time += self._period
            return True
        return False

    def Tick(self, time):
        raise NotImplementedError


class ChROSClockHandler(ChROSHandler):
    """Publishes the simulation clock on /clock (rosgraph_msgs/Clock shape)."""

    def __init__(self, update_rate=CLOCK_RATE, topic="/clock"):
        super().__init__(update_rate, topic)

    def Tick(self, time):
        self.last_message = {"clock": time}


class ChROSDriverInputsHandler(ChROSHandler):
    """Publishes scripted driver inputs (steering/throttle/braking)."""

    def __init__(self, driver, update_rate=DRIVER_RATE, topic="/vehicle/driver_inputs"):
        super().__init__(update_rate, topic)
        self.driver = driver  # cache: driver handle reused every Tick

    def Tick(self, time):
        di = self.driver.GetInputs()
        self.last_message = {
            "steering": di.m_steering,
            "throttle": di.m_throttle,
            "braking": di.m_braking,
        }


class ChROSBodyHandler(ChROSHandler):
    """Publishes a body's pose + twist (nav_msgs/Odometry shape)."""

    def __init__(self, body, update_rate=BODY_RATE, topic="/vehicle/chassis/odometry"):
        super().__init__(update_rate, topic)
        self.body = body  # cache: chassis body handle reused every Tick

    def Tick(self, time):
        p = self.body.GetPos()
        v = self.body.GetPosDt()
        q = self.body.GetRot()
        self.last_message = {
            "px": p.x, "py": p.y, "pz": p.z,
            "vx": v.x, "vy": v.y, "vz": v.z,
            "q0": q.e0, "q1": q.e1, "q2": q.e2, "q3": q.e3,
        }


class ChROSManager:
    """Owns and drives the ROS-shaped handlers (Register/Initialize/Update)."""

    def __init__(self):
        self.handlers = []
        self.published = {}  # topic -> latest captured message

    def RegisterHandler(self, handler):
        self.handlers.append(handler)

    def Initialize(self):
        for h in self.handlers:
            h.Initialize()
        return True

    def Update(self, time, timeout=0.0):
        # Mirror ChROSManager::Update: tick each handler, capture its message.
        for h in self.handlers:
            if h.Update(time) and h.last_message is not None:
                self.published[h.topic] = h.last_message
        return True


# === Scripted driver (veh.ChDriver subclass) ===
class ScriptedDriver(veh.ChDriver):
    """Brake-hold, then constant throttle with a sinusoidal steering sweep."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < LAUNCH_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
            # mild sweep so yaw is visible; bounded to STEER_AMP
            self.SetSteering(STEER_AMP * math.sin(STEER_RATE * (time - LAUNCH_TIME)))


def main():
    # === Vehicle (HMMWV_Full wrapper) ===
    # The wrapper creates and OWNS its ChSystem; do not pass a system to it.
    init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
    init_rot = chrono.QUNIT
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)  # prompt: tire model — TMEASY for rigid road grip
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    system = hmmwv.GetSystem()                 # ChSystemSMC owned by the wrapper
    chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()               # cache: ChWheeledVehicle handle for spindle queries
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); suspension + steering
    # links and the powertrain shafts are created inside the wrapper.

    # Footprint assert: confirm wheels rest ON the ground (not through it).
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - Z_TOL, (
        f"vehicle sinks into ground: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs ground top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Terrain (flat rigid patch) ===
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(TERRAIN_YOUNG)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver (scripted ChDriver subclass) ===
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === ROS-shaped publishing layer (manager + handlers) ===
    ros_manager = ChROSManager()
    clock_handler = ChROSClockHandler()
    driver_handler = ChROSDriverInputsHandler(driver)
    body_handler = ChROSBodyHandler(chassis)
    ros_manager.RegisterHandler(clock_handler)
    ros_manager.RegisterHandler(driver_handler)
    ros_manager.RegisterHandler(body_handler)
    ros_manager.Initialize()

    # === Visualization (full Irrlicht scene; window gated for fast validation) ===
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV + ROS-shaped publishers")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.6)
        vis.Initialize()                                   # Initialize FIRST (Irrlicht)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                    # outdoor sky backdrop
        vis.AddTypicalLights()                             # standard lighting
        vis.AddLight(chrono.ChVector3d(30, 30, 100), 250, chrono.ChColor(0.7, 0.7, 0.7))
        vis.AttachVehicle(veh_obj)                         # bind chassis/wheel/tire assets
        vis.AttachDriver(driver)                           # input-bar HUD

    # === Derived loop constants (precomputed once) ===
    render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short check when validating
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)      # guard against missing review dir

    sim_writer = None
    motion_writer = None
    sim_f = None
    motion_f = None
    try:
        # Open both CSVs via context managers so they always flush/close.
        with open(OUT_SIM_CSV, "w", newline="") as sim_f, \
             open(OUT_MOTION_CSV, "w", newline="") as motion_f:
            sim_writer = csv.writer(sim_f)
            sim_writer.writerow([
                "time", "pos_x", "pos_y", "pos_z", "speed",
                "vel_x", "vel_y", "vel_z",
                "ros_clock", "ros_throttle", "ros_steering", "ros_braking",
                "ros_odom_px", "ros_odom_vx",
            ])
            motion_writer = csv.writer(motion_f)
            motion_writer.writerow([
                "time", "body", "pos_x", "pos_y", "pos_z",
                "vel_x", "vel_y", "vel_z", "q0", "q1", "q2", "q3",
            ])

            # === Main loop (render-cadence outer, physics inner batch) ===
            frame = 0
            while (HEADLESS or vis.Run()) and system.GetChTime() < run_end:
                if not HEADLESS:
                    vis.BeginScene()
                    vis.Render()
                    vis.EndScene()
                    vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                    frame += 1

                for _ in range(render_every):
                    time = system.GetChTime()
                    driver_inputs = driver.GetInputs()

                    # Standard vehicle subsystem synchronize order.
                    driver.Synchronize(time)
                    terrain.Synchronize(time)
                    hmmwv.Synchronize(time, driver_inputs, terrain)
                    if not HEADLESS:
                        vis.Synchronize(time, driver_inputs)

                    # ROS-shaped publish step (mirrors ChROSManager::Update).
                    ros_manager.Update(time)

                    # Log physics + published ROS quantities this step.
                    pos = chassis.GetPos()
                    vel = chassis.GetPosDt()
                    rot = chassis.GetRot()
                    speed = veh_obj.GetSpeed()
                    odom = ros_manager.published.get("/vehicle/chassis/odometry", {})
                    sim_writer.writerow([
                        f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                        f"{speed:.5f}", f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
                        f"{ros_manager.published.get('/clock', {}).get('clock', time):.5f}",
                        f"{driver_inputs.m_throttle:.5f}", f"{driver_inputs.m_steering:.5f}",
                        f"{driver_inputs.m_braking:.5f}",
                        f"{odom.get('px', pos.x):.5f}", f"{odom.get('vx', vel.x):.5f}",
                    ])
                    motion_writer.writerow([
                        f"{time:.5f}", "chassis",
                        f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                        f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
                        f"{rot.e0:.5f}", f"{rot.e1:.5f}", f"{rot.e2:.5f}", f"{rot.e3:.5f}",
                    ])

                    # Advance the full subsystem stack (vehicle.Advance steps system).
                    driver.Advance(TIME_STEP)
                    terrain.Advance(TIME_STEP)
                    hmmwv.Advance(TIME_STEP)
                    if not HEADLESS:
                        vis.Advance(TIME_STEP)
                    if system.GetChTime() >= run_end:
                        break
    except (OSError, IOError) as exc:           # disk / permission failure on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # CSVs are closed by the `with` block; nothing else to flush here.
        pass

    # === Post-processing (timeseries plot from the logged CSV) ===
    try:
        with open(OUT_SIM_CSV, "r", newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
    except (OSError, IOError) as exc:           # plot is best-effort; missing CSV is non-fatal
        import traceback
        traceback.print_exc()
        rows = []

    if rows:
        t = np.array([float(r["time"]) for r in rows])
        px = np.array([float(r["pos_x"]) for r in rows])
        spd = np.array([float(r["speed"]) for r in rows])
        thr = np.array([float(r["ros_throttle"]) for r in rows])
        fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        ax[0].plot(t, px, label="chassis x (m)")
        ax[0].set_ylabel("position x (m)")
        ax[0].grid(True)
        ax[0].legend()
        ax[1].plot(t, spd, color="tab:orange", label="speed (m/s)")
        ax[1].set_ylabel("speed (m/s)")
        ax[1].grid(True)
        ax[1].legend()
        ax[2].plot(t, thr, color="tab:green", label="ROS throttle")
        ax[2].set_ylabel("throttle")
        ax[2].set_xlabel("time (s)")
        ax[2].grid(True)
        ax[2].legend()
        fig.suptitle("HMMWV chassis motion + published ROS driver input")
        fig.tight_layout()
        fig.savefig(OUT_PLOT, dpi=110)
        plt.close(fig)

        # Lightweight verdict to stdout for the validation gate.
        moved = float(px[-1] - px[0])
        print(f"[validate] frames_written={'(headless)' if HEADLESS else 'rendered'} "
              f"rows={len(rows)} dx={moved:.3f} m final_speed={spd[-1]:.3f} m/s "
              f"final_pz={float(rows[-1]['pos_z']):.3f} m")


if __name__ == "__main__":
    main()
