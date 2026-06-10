"""Full HMMWV on flat rigid terrain — PyChrono 9.0.1 + Irrlicht.

Model
-----
A complete HMMWV (High Mobility Multipurpose Wheeled Vehicle) catalog model
(`veh.HMMWV_Full`) driving on a flat `veh.RigidTerrain` patch. The vehicle owns
its own `ChSystemSMC` (SMC contact, created internally by the wrapper); the
terrain, driver, and visualization are attached to that same owned system.

System / contact
----------------
SMC (penalty) contact via `chrono.ChContactMethod_SMC`. Z-up world, gravity
-9.81 m/s^2 along world Z (set by the wrapper).

Main bodies
-----------
- chassis: the HMMWV body (geometric-center chassis-frame origin).
- four wheel spindles (front-left/right, rear-left/right) created by the wrapper.
- terrain patch: a single flat rigid box patch (RigidTerrain), textured.
The suspension, steering, and driveline links are created inside the wrapper.

Tire / driveline
----------------
TMEASY tire model (`veh.TireModelType_TMEASY`), SHAFTS engine + automatic-shafts
transmission, all-wheel drive, Pitman-arm steering. Vehicle subsystems are drawn
with PRIMITIVE visualization shapes per the request.

Driver
------
A scripted driver (subclass of `veh.ChDriver`) provides steering / throttle /
braking inputs over time (an autonomous stand-in for an interactive driver, so
the run is reproducible headless): brief settle, then accelerate forward while
applying a gentle sinusoidal steer, then ease off and brake near the end.

Expected behaviour
------------------
The chassis should rest on the terrain at spawn, then accelerate and translate
forward (X increases, speed rises above ~3 m/s) while gently turning, then slow
as the brake is applied. Logged to CSV and rendered at 50 fps for review.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")            # headless-safe plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / control) ===
TIME_STEP = 1.0e-3               # integration step (s)
TIRE_STEP = 1.0e-3               # TMEASY tire substep (s)
SIM_END = 12.0                   # simulation duration (s)
RENDER_FPS = 50.0                # real-time review cadence (frames per second)

INIT_X, INIT_Y = 0.0, 0.0        # spawn X/Y (m), world frame
SUSPENSION_REF_HEIGHT = 0.5      # HMMWV chassis-origin height above wheel-bottom at rest (m)
TERRAIN_TOP_Z = 0.0              # flat rigid terrain surface height (m)
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived chassis spawn Z (m)

TERRAIN_LENGTH = 200.0           # rigid terrain patch size along X (m)
TERRAIN_WIDTH = 100.0            # rigid terrain patch size along Y (m)
TERRAIN_FRICTION = 0.9           # tire/ground friction coefficient
TERRAIN_RESTITUTION = 0.01       # ground restitution
TERRAIN_YOUNG = 2.0e7            # SMC contact stiffness (Pa)

ZTOL = 0.10                      # tolerance for wheel-bottom-on-terrain assert (m)

# Scripted-driver schedule (s) — settle, drive, then brake.
T_SETTLE = 0.5                   # hold still while suspension settles
T_BRAKE = 10.0                   # begin braking after this time
THROTTLE_DRIVE = 0.7             # steady throttle during the drive phase
STEER_AMPLITUDE = 0.25           # gentle steering amplitude (-1..1)
STEER_RATE = 0.4                 # steering oscillation rate (rad/s)

# === Scripted driver (autonomous steering / throttle / braking) ===
# A ChDriver subclass that sets steering, throttle, and braking from a
# time-based control law inside Synchronize(); base GetInputs() returns these.
class ScriptedDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < T_SETTLE:                      # let the chassis settle on its tires
            self.SetThrottle(0.0)
            self.SetBraking(0.0)
            self.SetSteering(0.0)
        elif time < T_BRAKE:                     # accelerate forward + gentle steer
            self.SetThrottle(THROTTLE_DRIVE)
            self.SetBraking(0.0)
            self.SetSteering(STEER_AMPLITUDE * math.sin(STEER_RATE * (time - T_SETTLE)))
        else:                                    # ease off throttle and brake to a stop
            self.SetThrottle(0.0)
            self.SetBraking(0.8)
            self.SetSteering(0.0)


HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast windowless validation run
RUN_END = min(SIM_END, 1.0) if HEADLESS else SIM_END   # short physics check when validating

# Derived render cadence (precomputed once; never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # physics steps per frame

os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
os.makedirs("cam", exist_ok=True)      # guard against missing motion-log/video dir

# CSV file handles declared up front so `finally` can always flush/close them.
data_f = None
motion_f = None

try:
    # === Vehicle (HMMWV_Full wrapper: TMEASY tire, SMC contact, AWD) ===
    init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)            # identity: facing +X

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)    # prompt: contact method
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))  # prompt: location + orientation
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)           # prompt: TMEASY tire model
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    # Primitive visualization for the vehicle components (prompt request).
    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    sys = hmmwv.GetSystem()                    # cache: ChSystemSMC owned by the wrapper, reused every step
    veh_obj = hmmwv.GetVehicle()               # cache: ChWheeledVehicle, reused every step
    chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side) for the four spindles below.
    # joints: suspension + Pitman-arm steering + driveline links built inside the wrapper.

    # Assert the wheels rest on (not through) the flat terrain after Initialize.
    n_axles = veh_obj.GetNumberAxles()
    spindle_world = []
    for axle in range(n_axles):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    tire_radius = veh_obj.GetAxle(0).GetWheels()[0].GetTire().GetRadius()
    wheel_bottom_z = min(p.z for p in spindle_world) - tire_radius
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Terrain (flat textured RigidTerrain patch) ===
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(TERRAIN_YOUNG)

    terrain = veh.RigidTerrain(sys)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver (scripted steering / throttle / braking) ===
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full vehicle Irrlicht scene: window + sky + chase camera + lights
    # Gated behind HEADLESS so a validation run skips the on-screen window for speed;
    # the full standard setup below is what the renderer uses for the review video.
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV on Rigid Terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)  # follow the chassis
        vis.Initialize()                                                  # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()                                                   # standard sky backdrop
        vis.AddTypicalLights()                                            # standard lighting
        vis.AttachVehicle(veh_obj)                                        # bind chassis/wheel assets
        vis.AttachDriver(driver)                                          # steering/throttle HUD bars

    # === Logging setup (open CSVs with context managers; flushed in finally) ===
    try:
        data_f = open("simulation_data.csv", "w", newline="")            # main physics log
        motion_f = open("cam/motion_log.csv", "w", newline="")           # chassis pose/velocity log
    except (OSError, IOError) as exc:                                     # disk full / permission denied
        print(f"failed to open CSV output: {exc}")
        raise

    data_writer = csv.writer(data_f)
    data_writer.writerow(["time", "x", "y", "z", "speed", "throttle", "steering", "braking"])
    motion_writer = csv.writer(motion_f)
    motion_writer.writerow(["time", "px", "py", "pz", "vx", "vy", "vz", "speed"])

    times, speeds, xs, throttles = [], [], [], []

    # === Main loop (render-cadence outer loop; Synchronize/Advance inner batch) ===
    # vis.Advance() steps the wrapper-owned ChSystem, so we never call
    # sys.DoStepDynamics() here (that would double-step the subsystems).
    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")          # consecutive index -> ffmpeg
            frame += 1

        for _ in range(RENDER_EVERY):
            time = sys.GetChTime()
            driver_inputs = driver.GetInputs()

            # --- log physics every step ---
            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            speed = veh_obj.GetSpeed()
            data_writer.writerow([
                f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                f"{speed:.5f}", f"{driver_inputs.m_throttle:.4f}",
                f"{driver_inputs.m_steering:.4f}", f"{driver_inputs.m_braking:.4f}",
            ])
            motion_writer.writerow([
                f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}", f"{speed:.5f}",
            ])
            times.append(time)
            speeds.append(speed)
            xs.append(pos.x)
            throttles.append(driver_inputs.m_throttle)

            # --- advance the full subsystem stack (driver/terrain/vehicle/vis) ---
            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)            # internally steps the wrapper-owned system
            if not HEADLESS:
                vis.Advance(TIME_STEP)

            if sys.GetChTime() >= RUN_END:
                break

except (RuntimeError, ValueError) as exc:        # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    # === Post-processing === flush + close CSV writers even on early exit
    if data_f is not None:
        data_f.close()
    if motion_f is not None:
        motion_f.close()

# === Post-processing: timeseries plot ===
if times:
    t = np.array(times)
    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(t, speeds, color="tab:blue")
    ax[0].set_ylabel("speed (m/s)")
    ax[0].grid(True)
    ax[1].plot(t, xs, color="tab:green")
    ax[1].set_ylabel("chassis x (m)")
    ax[1].grid(True)
    ax[2].plot(t, throttles, color="tab:red")
    ax[2].set_ylabel("throttle")
    ax[2].set_xlabel("time (s)")
    ax[2].grid(True)
    fig.suptitle("HMMWV on rigid terrain — motion")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)
    print(f"final time={t[-1]:.3f}s  final x={xs[-1]:.3f}m  max speed={max(speeds):.3f}m/s")
