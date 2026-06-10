"""Two-sedan rigid-road driving simulation (PyChrono 9.0.1, Irrlicht).

Models two catalog `veh.Sedan` wheeled vehicles spawned side-by-side on a flat
rigid terrain patch textured with concrete. Each vehicle owns a TMEASY tire model
and is controlled by an independent scripted driver that applies a constant
forward throttle and a sinusoidal steering signal, so both cars drive forward
while weaving left and right.

System type: NSC (Non-Smooth Contact) — the wrapper-owned ChSystemNSC created by
the first Sedan wrapper; the second vehicle and the terrain share that same system.
Main bodies: vehicle A chassis + 4 wheels, vehicle B chassis + 4 wheels, and one
fixed rigid terrain patch. Expected behavior: both sedans translate forward along
+X while their headings oscillate from the sinusoidal steering input; neither
tips over and both stay on the terrain.
"""

# === Imports (public PyChrono + stdlib only; self-contained) ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / control) ===
TIME_STEP = 1.0e-3                 # integration step (s)
SIM_END = 8.0                      # total simulated time (s)
RENDER_FPS = 50.0                  # review-video frame rate
TIRE_STEP_SIZE = 1.0e-3            # TMEASY tire substep (s)

TERRAIN_LENGTH = 200.0             # X extent of rigid patch (m) — sized to driven distance
TERRAIN_WIDTH = 100.0              # Y extent of rigid patch (m)
TERRAIN_FRICTION = 0.9             # concrete-road friction
TERRAIN_RESTITUTION = 0.01         # near-inelastic road contact

CHASSIS_INIT_Z = 0.5               # Sedan chassis-origin rest height above flat ground
VEH_A_INIT = chrono.ChVector3d(-18.0, 5.0, CHASSIS_INIT_Z)   # first sedan spawn (far lane)
VEH_B_INIT = chrono.ChVector3d(-18.0, -5.0, CHASSIS_INIT_Z)  # second sedan spawn (near lane)
VEH_A_YAW = 0.0                    # both face +X (drive across the static camera view)
VEH_B_YAW = 0.0

DRIVE_THROTTLE = 0.22              # gentle forward throttle (cars cross the framed stretch in 8 s)
STEER_AMPLITUDE = 0.55             # peak |steering| of the sinusoid (-1..+1) — pronounced weave
STEER_FREQ_A = 0.45                # steering frequency, vehicle A (Hz)
STEER_FREQ_B = 0.65                # steering frequency, vehicle B (distinct weave)
THROTTLE_RAMP = 1.0                # ramp throttle in over first second (no wheel-spin launch)

TIRE_RADIUS = 0.3266               # Sedan TMEASY tire radius (m) — from wheel geometry
GROUND_TOP_Z = 0.0                 # flat rigid patch top surface
ZTOL = 0.05                        # allowed wheel-bottom clearance/overlap vs ground

# FIXED camera (does NOT follow the cars): an elevated 3/4 view of the driving
# corridor so the cars visibly translate across the static grid and their lateral
# weave reads clearly against the fixed background.
CAM_EYE_X = -22.0                  # eye behind the start, off to the near side
CAM_EYE_Y = -28.0                  # eye Y offset for a 3/4 side view of both lanes
CAM_EYE_Z = 14.0                   # eye height
CAM_TGT_X = 8.0                    # look-at: centre of the driven corridor
CAM_TGT_Y = 0.0                    # look-at Y: between the two lanes
CAM_TGT_Z = 0.5


# === Scripted sinusoidal driver (subclass of veh.ChDriver) ===
class SinusoidDriver(veh.ChDriver):
    """Constant ramped throttle + sinusoidal steering, driven by sim time."""

    def __init__(self, vehicle, steer_amp, steer_freq):
        super().__init__(vehicle)
        self.steer_amp = steer_amp          # cache: control params stored once
        self.steer_freq = steer_freq

    def Synchronize(self, time):
        # Throttle ramps linearly to DRIVE_THROTTLE over THROTTLE_RAMP seconds.
        throttle = DRIVE_THROTTLE * min(1.0, time / THROTTLE_RAMP)
        self.SetThrottle(throttle)
        self.SetBraking(0.0)
        # Sinusoidal steering: cars weave left/right while driving forward.
        self.SetSteering(self.steer_amp * math.sin(2.0 * math.pi * self.steer_freq * time))


def build_sedan(init_pos, yaw):
    """Create + initialize a catalog Sedan with TMEASY tires at init_pos."""
    sedan = veh.Sedan()
    sedan.SetContactMethod(chrono.ChContactMethod_NSC)
    sedan.SetChassisCollisionType(veh.CollisionType_NONE)
    sedan.SetChassisFixed(False)
    sedan.SetInitPosition(
        chrono.ChCoordsysd(init_pos, chrono.QuatFromAngleZ(yaw))
    )
    sedan.SetTireType(veh.TireModelType_TMEASY)   # prompt: forward-driving tire on rigid road
    sedan.SetTireStepSize(TIRE_STEP_SIZE)
    sedan.Initialize()
    sedan.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    sedan.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    sedan.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    sedan.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    sedan.SetTireVisualizationType(chrono.VisualizationType_MESH)
    return sedan


# === Headless validation gate ===
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run

# === Output directories ===
os.makedirs("frames", exist_ok=True)   # guard: ffmpeg frame source dir
os.makedirs("cam", exist_ok=True)       # guard: review video + motion log dir

# === Precomputed loop constants (once, before the loop) ===
render_step_size = 1.0 / RENDER_FPS                       # precomputed once
render_steps = max(1, math.ceil(render_step_size / TIME_STEP))   # precomputed once
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END      # short physics check when validating

# Handles populated during build; closed/flushed in finally.
data_file = None
motion_file = None

try:
    # === System & bodies (vehicle A's wrapper owns the ChSystemNSC; B + terrain share it) ===
    vehicle_a = build_sedan(VEH_A_INIT, VEH_A_YAW)
    sys = vehicle_a.GetSystem()                 # ChSystemNSC owned by the first wrapper
    vehicle_b = build_sedan(VEH_B_INIT, VEH_B_YAW)   # second sedan registers into the same system

    veh_a_obj = vehicle_a.GetVehicle()          # cache: vehicle subsystem handle, reused below
    veh_b_obj = vehicle_b.GetVehicle()          # cache: vehicle subsystem handle, reused below
    chassis_a = vehicle_a.GetChassisBody()      # cache: main chassis body A, reused every step
    chassis_b = vehicle_b.GetChassisBody()      # cache: main chassis body B, reused every step
    # wheels/spindles: veh_*_obj.GetSpindlePos(axle, side); suspension + steering joints
    # are created inside each Sedan wrapper.

    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    # === Footprint asserts (wheels rest on the flat ground, not through it) ===
    for vo in (veh_a_obj, veh_b_obj):
        spindle_world = []
        for axle in range(vo.GetNumberAxles()):
            for side in (veh.LEFT, veh.RIGHT):
                spindle_world.append(vo.GetSpindlePos(axle, side))
        wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
        assert wheel_bottom_z >= GROUND_TOP_Z - ZTOL, (
            f"vehicle sinks into ground: wheel bottom z={wheel_bottom_z:.3f} "
            f"vs ground top z={GROUND_TOP_Z:.3f}; raise CHASSIS_INIT_Z by "
            f"{GROUND_TOP_Z - wheel_bottom_z:.3f} m"
        )

    # === Terrain (flat rigid patch, concrete texture) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(
        veh.GetVehicleDataFile("terrain/textures/concrete.jpg"), 200, 200
    )
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.Initialize()

    # === Drivers (one independent sinusoidal driver per vehicle) ===
    driver_a = SinusoidDriver(veh_a_obj, STEER_AMPLITUDE, STEER_FREQ_A)
    driver_b = SinusoidDriver(veh_b_obj, STEER_AMPLITUDE, STEER_FREQ_B)
    driver_a.Initialize()
    driver_b.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + fixed camera + lights + grid
    # A generic Irrlicht system with a FIXED elevated camera is used (not a chase camera)
    # so both sedans and their side-to-side weave stay framed instead of being tracked.
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)                             # binds both vehicles + terrain assets
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Two Sedans — sinusoidal steering on concrete")
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
        vis.Initialize()                                  # Initialize FIRST (Irrlicht order)
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()                                   # standard outdoor sky backdrop
        vis.AddCamera(chrono.ChVector3d(CAM_EYE_X, CAM_EYE_Y, CAM_EYE_Z),
                      chrono.ChVector3d(CAM_TGT_X, CAM_TGT_Y, CAM_TGT_Z))   # AFTER Initialize
        vis.AddTypicalLights()                            # standard lighting
        vis.AddGrid(2.0, 2.0, 80, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))        # ground reference grid

    # === CSV writers (context-managed so they always flush/close) ===
    try:
        data_file = open("simulation_data.csv", "w", newline="")          # disk/permission guard
        motion_file = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:   # disk full / permission denied
        print(f"failed to open CSV output: {exc}")
        raise

    data_writer = csv.writer(data_file)
    data_writer.writerow([
        "time",
        "a_x", "a_y", "a_speed", "a_steer",
        "b_x", "b_y", "b_speed", "b_steer",
    ])
    motion_writer = csv.writer(motion_file)
    motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    # === Main loop (render-cadence outer loop; physics in inner batch) ===
    step = 0
    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
        if not HEADLESS and (step % render_steps == 0):
            # Static camera (set once at build): the cars translate across the fixed
            # view, making forward motion and the side-to-side weave directly visible.
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
            frame += 1

        time = sys.GetChTime()
        inputs_a = driver_a.GetInputs()
        inputs_b = driver_b.GetInputs()

        # Log physics each step.
        speed_a = veh_a_obj.GetSpeed()
        speed_b = veh_b_obj.GetSpeed()
        pos_a = chassis_a.GetPos()
        pos_b = chassis_b.GetPos()
        vel_a = chassis_a.GetPosDt()
        vel_b = chassis_b.GetPosDt()
        data_writer.writerow([
            f"{time:.5f}",
            f"{pos_a.x:.5f}", f"{pos_a.y:.5f}", f"{speed_a:.5f}", f"{inputs_a.m_steering:.5f}",
            f"{pos_b.x:.5f}", f"{pos_b.y:.5f}", f"{speed_b:.5f}", f"{inputs_b.m_steering:.5f}",
        ])
        motion_writer.writerow([f"{time:.5f}", "sedan_a",
                                f"{pos_a.x:.5f}", f"{pos_a.y:.5f}", f"{pos_a.z:.5f}",
                                f"{vel_a.x:.5f}", f"{vel_a.y:.5f}", f"{vel_a.z:.5f}"])
        motion_writer.writerow([f"{time:.5f}", "sedan_b",
                                f"{pos_b.x:.5f}", f"{pos_b.y:.5f}", f"{pos_b.z:.5f}",
                                f"{vel_b.x:.5f}", f"{vel_b.y:.5f}", f"{vel_b.z:.5f}"])

        # Synchronize all subsystems for BOTH vehicles + their drivers.
        driver_a.Synchronize(time)
        driver_b.Synchronize(time)
        terrain.Synchronize(time)
        vehicle_a.Synchronize(time, inputs_a, terrain)
        vehicle_b.Synchronize(time, inputs_b, terrain)

        # Advance all subsystems by one step (vehicle.Advance steps the shared system).
        driver_a.Advance(TIME_STEP)
        driver_b.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        vehicle_a.Advance(TIME_STEP)
        vehicle_b.Advance(TIME_STEP)
        step += 1

except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close any open writers even if a step diverged mid-run.
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing (plot logged time series) ===
try:
    with open("simulation_data.csv", "r", newline="") as f:   # context-managed read
        rows = list(csv.reader(f))
except (OSError, IOError) as exc:   # missing/locked CSV
    print(f"could not read simulation_data.csv for plotting: {exc}")
    rows = []

if len(rows) > 1:
    header = rows[0]
    data = np.array([[float(v) for v in r] for r in rows[1:]])
    col = {name: i for i, name in enumerate(header)}   # cache: column index map
    t = data[:, col["time"]]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
    ax1.plot(t, data[:, col["a_x"]], label="sedan A x")
    ax1.plot(t, data[:, col["b_x"]], label="sedan B x")
    ax1.plot(t, data[:, col["a_y"]], "--", label="sedan A y")
    ax1.plot(t, data[:, col["b_y"]], "--", label="sedan B y")
    ax1.set_ylabel("position (m)")
    ax1.legend(loc="best")
    ax1.grid(True)

    ax2.plot(t, data[:, col["a_speed"]], label="sedan A speed")
    ax2.plot(t, data[:, col["b_speed"]], label="sedan B speed")
    ax2.plot(t, data[:, col["a_steer"]], "--", label="sedan A steer")
    ax2.plot(t, data[:, col["b_steer"]], "--", label="sedan B steer")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("speed (m/s) / steering")
    ax2.legend(loc="best")
    ax2.grid(True)

    fig.suptitle("Two sedans — sinusoidal steering on concrete road")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    print("wrote simulation_timeseries.png")

print("simulation complete")
