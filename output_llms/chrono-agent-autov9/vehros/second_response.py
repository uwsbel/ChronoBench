"""HMMWV wheeled vehicle with a ROS publisher framework, visualized in Irrlicht.

Model
-----
A full-model HMMWV (`veh.HMMWV_Full`) drives on a flat rigid terrain patch. The
vehicle wrapper owns a single `ChSystemNSC` (NSC contact). The chassis, the four
wheel spindles, the suspension/steering joints, and the powertrain are all created
internally by the wrapper; we fetch named handles to them so the essential bodies
are explicit. A scripted `veh.ChDriver` subclass applies a steady forward throttle
with a small-amplitude, net-zero sinusoidal steering signal so the vehicle stays on
the large terrain patch.

ROS substitution
----------------
This PyChrono build does not ship the optional `pychrono.ros` bindings, so the ROS
publishing layer is reconstructed as a self-contained, pure-Python framework that
mirrors the pychrono.ros API SHAPE: a manager (`ChROSManager`) owning a set of
handlers (`ChROSClockHandler`, a driver-inputs handler, and a `ChROSBodyHandler`).
Each handler exposes `Initialize()` and `Update(time)`; the manager registers them,
initializes them once, and ticks `Update` every step. Instead of emitting real DDS
messages, the handlers publish into in-process topic buffers that we log to CSV — the
control/data flow and registration lifecycle are identical to the real ROS bridge.

System type: NSC (rigid terrain, rigid-body contact via the HMMWV wrapper).
Expected behavior: the vehicle accelerates forward in +X, the chase camera follows
it, and the published clock / driver-input / chassis-pose topics are logged to CSV.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")                      # headless-safe plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / control) ===
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation gate

TIME_STEP = 2.0e-3                         # integration step (s)
TIRE_STEP = 1.0e-3                         # tire substep (s)
SIM_END = 12.0                             # total simulated time (s)
RENDER_FPS = 30.0                          # review-video frame rate
ROS_PUBLISH_RATE = 25.0                    # Hz, ROS-style topic publish cadence

TERRAIN_LENGTH = 200.0                     # rigid patch X extent (m) — large, vehicle stays on
TERRAIN_WIDTH = 200.0                      # rigid patch Y extent (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_YOUNG = 2.0e7                      # SMC-style stiffness (unused by NSC patch but kept explicit)

INIT_X, INIT_Y = -80.0, 0.0               # spawn near one end so there is room to drive
SUSPENSION_REF_HEIGHT = 0.5                # HMMWV chassis-origin height above wheel-bottom at rest
TERRAIN_TOP_Z = 0.0                        # flat patch top plane
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT
ZTOL = 0.10                                # allowed wheel-bottom clearance vs support

THROTTLE_RAMP_END = 1.0                    # brief brake-hold before driving (s)
CRUISE_THROTTLE = 0.6                      # steady forward throttle
STEER_AMPLITUDE = 0.06                     # small steering amplitude (net-zero sine)
STEER_PERIOD = 6.0                         # steering oscillation period (s)

# Derived constants (precomputed once — never recomputed in the hot loop)
RENDER_STEPS = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))    # physics steps per frame
PUBLISH_STEPS = max(1, round(1.0 / (ROS_PUBLISH_RATE * TIME_STEP)))  # steps per ROS publish
RUN_END = min(SIM_END, 1.0) if HEADLESS else SIM_END           # short physics check when validating
STEER_OMEGA = 2.0 * math.pi / STEER_PERIOD                     # precomputed angular rate


# === ROS publisher framework (pychrono.ros API shape, in-process substitution) ===
# Mirrors ChROSManager + ChROSClockHandler + a driver-inputs handler + ChROSBodyHandler.
# Each handler has Initialize()/Update(time); the manager Register/Initialize/Update each step.
class ChROSHandler:
    """Base handler — publish-rate throttled, like the real ChROSHandler."""

    def __init__(self, update_rate, topic):
        self._update_rate = float(update_rate)        # Hz
        self._topic = topic
        self._last_publish = -1.0
        self.last_message = None                       # latest published payload (in-process)

    def _due(self, time):
        # cache: publish period computed from the configured rate
        period = 1.0 / self._update_rate if self._update_rate > 0 else 0.0
        return self._last_publish < 0.0 or (time - self._last_publish) >= period

    def Initialize(self):
        return True

    def Update(self, time):
        raise NotImplementedError


class ChROSClockHandler(ChROSHandler):
    """Publishes simulation time on /clock (mirrors pychrono.ros ChROSClockHandler)."""

    def __init__(self, update_rate=1.0e3, topic="/clock"):
        super().__init__(update_rate, topic)

    def Update(self, time):
        if self._due(time):
            self.last_message = {"clock": time}        # rosgraph_msgs/Clock shape
            self._last_publish = time


class ChROSDriverInputsHandler(ChROSHandler):
    """Publishes the latest driver inputs (steering/throttle/braking) on a topic."""

    def __init__(self, driver, update_rate=ROS_PUBLISH_RATE, topic="/vehicle/driver_inputs"):
        super().__init__(update_rate, topic)
        self._driver = driver                          # cache: driver reused every Update

    def Update(self, time):
        if self._due(time):
            di = self._driver.GetInputs()              # current DriverInputs struct
            self.last_message = {
                "steering": di.m_steering,
                "throttle": di.m_throttle,
                "braking": di.m_braking,
            }
            self._last_publish = time


class ChROSBodyHandler(ChROSHandler):
    """Publishes a body pose + linear velocity on a topic (mirrors ChROSBodyHandler)."""

    def __init__(self, body, update_rate=ROS_PUBLISH_RATE, topic="/vehicle/chassis_state"):
        super().__init__(update_rate, topic)
        self._body = body                              # cache: body handle reused every Update

    def Update(self, time):
        if self._due(time):
            p = self._body.GetPos()
            v = self._body.GetPosDt()
            self.last_message = {
                "x": p.x, "y": p.y, "z": p.z,
                "vx": v.x, "vy": v.y, "vz": v.z,
            }
            self._last_publish = time


class ChROSManager:
    """Registers handlers, initializes them once, and ticks Update each step."""

    def __init__(self):
        self._handlers = []

    def RegisterHandler(self, handler):
        self._handlers.append(handler)

    def Initialize(self):
        for h in self._handlers:
            h.Initialize()

    def Update(self, time, step):
        # Tick every registered handler; each self-throttles to its publish rate.
        for h in self._handlers:
            h.Update(time)
        return True


# === Scripted driver (small-amplitude, net-zero steering keeps vehicle on patch) ===
class ScriptedDriver(veh.ChDriver):
    """Brief brake-hold, then steady throttle with a net-zero sinusoidal steer."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < THROTTLE_RAMP_END:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(CRUISE_THROTTLE)
            self.SetBraking(0.0)
        # net-zero over a full period -> vehicle holds its heading on the large patch
        self.SetSteering(STEER_AMPLITUDE * math.sin(STEER_OMEGA * time))


def main():
    # === Vehicle wrapper (creates + owns the ChSystemNSC, chassis, spindles, joints) ===
    veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")   # vehicle asset data path

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)    # slip/grip curve; drives reliably on rigid patch
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    sys = hmmwv.GetSystem()                         # ChSystemNSC owned by the wrapper
    chassis = hmmwv.GetChassisBody()                # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()                    # cache: ChWheeledVehicle for spindle queries
    # spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + Pitman steering inside wrapper

    # Footprint assert: wheels rest on (not through) the flat terrain after Initialize.
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    tire_radius = veh_obj.GetAxle(0).GetWheel(0, veh.LEFT).GetTire().GetRadius()
    wheel_bottom_z = min(p.z for p in spindle_world) - tire_radius
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into support: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Terrain (flat rigid patch, attached to the wrapper-owned system) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 100, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver (scripted; net-zero steering) ===
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === ROS publisher framework (manager + handlers; Register/Initialize/Update) ===
    ros_manager = ChROSManager()
    clock_handler = ChROSClockHandler()
    inputs_handler = ChROSDriverInputsHandler(driver)
    body_handler = ChROSBodyHandler(chassis)
    ros_manager.RegisterHandler(clock_handler)
    ros_manager.RegisterHandler(inputs_handler)
    ros_manager.RegisterHandler(body_handler)
    ros_manager.Initialize()

    # === Visualization (full Irrlicht vehicle scene: window + sky + camera + lights + grid) ===
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV with ROS publisher framework")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)  # follow the chassis
        vis.Initialize()                                            # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                             # outdoor sky backdrop
        vis.AddTypicalLights()                                      # standard lighting
        vis.AddGrid(2.0, 2.0, 50, 50,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))                  # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Output dirs + CSV writers ===
    os.makedirs("frames", exist_ok=True)            # guard against missing frame dir
    os.makedirs("cam", exist_ok=True)               # guard against missing cam dir

    sim_csv = None
    motion_csv = None
    try:
        sim_csv = open("simulation_data.csv", "w", newline="")          # main physics log
        motion_csv = open("cam/motion_log.csv", "w", newline="")        # per-body motion log
    except (OSError, IOError) as exc:               # disk / permission failure on open
        print(f"failed to open CSV outputs: {exc}")
        raise

    sim_writer = csv.writer(sim_csv)
    motion_writer = csv.writer(motion_csv)
    sim_writer.writerow([
        "time", "x", "y", "z", "speed", "throttle", "steering",
        "ros_clock", "ros_pub_x", "ros_pub_vx",
    ])
    motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    # Time-series buffers for the post-run plot
    t_hist, x_hist, speed_hist, throttle_hist, steer_hist = [], [], [], [], []

    # === Main loop (render-cadence outer; Synchronize/Advance inner — no DoStepDynamics) ===
    step = 0
    frame = 0
    try:
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_STEPS):
                time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # ROS-style publish tick (handlers self-throttle to ROS_PUBLISH_RATE)
                ros_manager.Update(time, step)

                # Log physics each step
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                pub = body_handler.last_message or {"x": pos.x, "vx": vel.x}
                clk = clock_handler.last_message["clock"] if clock_handler.last_message else time
                sim_writer.writerow([
                    f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{speed:.5f}", f"{driver_inputs.m_throttle:.4f}",
                    f"{driver_inputs.m_steering:.4f}",
                    f"{clk:.5f}", f"{pub['x']:.5f}", f"{pub['vx']:.5f}",
                ])
                motion_writer.writerow([
                    f"{time:.5f}", "chassis", f"{pos.x:.5f}", f"{pos.y:.5f}",
                    f"{pos.z:.5f}", f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
                ])
                if step % PUBLISH_STEPS == 0:
                    t_hist.append(time)
                    x_hist.append(pos.x)
                    speed_hist.append(speed)
                    throttle_hist.append(driver_inputs.m_throttle)
                    steer_hist.append(driver_inputs.m_steering)

                # Subsystem synchronize/advance order
                driver.Synchronize(time)
                terrain.Synchronize(time)
                hmmwv.Synchronize(time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)            # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)
                step += 1
                if sys.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:        # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close writers even if a step diverges mid-run.
        if sim_csv is not None:
            sim_csv.close()
        if motion_csv is not None:
            motion_csv.close()

    # === Post-processing (time-series plot from the logged data) ===
    if t_hist:
        fig, axs = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axs[0].plot(t_hist, x_hist, label="chassis x", color="tab:blue")
        axs[0].set_ylabel("x position (m)")
        axs[0].legend(loc="best")
        axs[0].grid(True)
        axs[1].plot(t_hist, speed_hist, label="speed", color="tab:green")
        axs[1].set_ylabel("speed (m/s)")
        axs[1].legend(loc="best")
        axs[1].grid(True)
        axs[2].plot(t_hist, throttle_hist, label="throttle", color="tab:red")
        axs[2].plot(t_hist, steer_hist, label="steering", color="tab:orange")
        axs[2].set_ylabel("driver inputs")
        axs[2].set_xlabel("time (s)")
        axs[2].legend(loc="best")
        axs[2].grid(True)
        fig.suptitle("HMMWV + ROS publisher framework — chassis motion & inputs")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    final_x = x_hist[-1] if x_hist else float("nan")
    print(f"done: steps={step} frames={frame} final_x={final_x:.3f} "
          f"final_speed={speed_hist[-1] if speed_hist else float('nan'):.3f}")


if __name__ == "__main__":
    main()
