"""M113 tracked vehicle mobility test on flat rigid terrain (PyChrono 9.0.1 + Irrlicht).

Models the M113 armored personnel carrier as a tracked vehicle driving forward
over flat rigid ground with a long box obstacle placed in its path to probe
mobility / track traction.

System type: NSC (non-smooth contact). A single-pin track shoe assembly is
unstable under SMC, so the vehicle, its terrain patch, and the obstacle box all
use an NSC system with NSC contact materials. Tractive torque is produced by a
SHAFTS engine + automatic SHAFTS transmission feeding a BDS (Bullet-Driveline
Simple) tracked driveline.

Main bodies:
  - M113 chassis + sprockets/idlers/road-wheels + left/right single-pin tracks
    (created internally by the veh.M113 wrapper).
  - A flat RigidTerrain patch (the support plane).
  - One long fixed box obstacle ("mobility_test_box") placed ahead of the spawn.

Expected behavior: the vehicle starts at (-5, 0, 0.5), settles onto the terrain,
and under a constant 0.8 throttle drives forward in +X, climbing toward / over
the long box obstacle. The chassis X position should increase monotonically and
the chassis should stay upright (Z up, no roll-over).
"""

import os
import math
import csv

import matplotlib
matplotlib.use("Agg")  # headless plotting backend (no display needed for PNG)
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / control) ===
TIME_STEP = 2e-3                      # solver step (s) — stable for NSC single-pin tracks
SIM_END = 8.0                         # total simulated time (s)
RENDER_FPS = 30.0                     # review-video frame rate

# Spawn pose (final desired values).
INIT_X = -5.0
INIT_Y = 0.0
INIT_Z = 0.5
INIT_LOC = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.QUNIT               # facing +X

# Constant control applied every step.
HARDCODED_THROTTLE = 0.8              # full-loop throttle command
STEERING_CMD = 0.0
BRAKING_CMD = 0.0

# Flat rigid terrain patch.
TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 40.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# Long box obstacle placed ahead of the vehicle to test mobility.
BOX_LEN_X = 8.0                       # long dimension, along travel direction
BOX_LEN_Y = 4.0
BOX_LEN_Z = 0.3                       # low ledge the tracks must climb
BOX_DENSITY = 1000.0
BOX_CENTER_X = 5.0                    # ahead of the spawn (+X)
BOX_CENTER_Y = 0.0
BOX_CENTER_Z = BOX_LEN_Z / 2.0        # rests on terrain top (z=0)
BOX_FRICTION = 0.9

# Chase-camera framing.
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.0)
CHASE_DISTANCE = 10.0
CHASE_HEIGHT = 1.5

# Derived constants — precomputed once (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))          # fast windowless validation
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short physics check when validating

# === System & bodies (M113 tracked vehicle wrapper) ===
# The veh.M113 wrapper creates and OWNS its ChSystemNSC plus the chassis,
# sprockets, idlers, road wheels, and both single-pin track assemblies. We
# configure powertrain/driveline before Initialize(), then fetch real handles.
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)          # NSC: stable single-pin tracks
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)             # tracked driveline -> sprocket torque
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.Initialize()

# Visualization types (after Initialize).
vehicle.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSprocketVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetIdlerVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetRoadWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetTrackShoeVisualizationType(chrono.VisualizationType_PRIMITIVES)

# Real wrapper-created handles (enumerated so the essentials are visible).
sys = vehicle.GetSystem()                 # cache: ChSystemNSC owned by the M113 wrapper
chassis = vehicle.GetChassisBody()        # cache: main chassis rigid body, reused every step
tracked = vehicle.GetVehicle()            # cache: ChTrackedVehicle (track-shoe / state API)
n_shoes_left = tracked.GetNumTrackShoes(veh.LEFT)    # precomputed once: per-side shoe count
n_shoes_right = tracked.GetNumTrackShoes(veh.RIGHT)  # precomputed once
# joints: sprocket/idler/road-wheel revolutes + track-shoe pin joints live inside the wrapper.

# === Terrain (flat rigid support patch) ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Mobility-test obstacle (one long box ahead of the spawn) ===
# Fixed long box the tracks must climb to demonstrate mobility / traction.
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(BOX_FRICTION)
box_mat.SetRestitution(0.0)
mobility_box = chrono.ChBodyEasyBox(
    BOX_LEN_X, BOX_LEN_Y, BOX_LEN_Z, BOX_DENSITY, True, True, box_mat
)
mobility_box.SetName("mobility_test_box")
mobility_box.SetPos(chrono.ChVector3d(BOX_CENTER_X, BOX_CENTER_Y, BOX_CENTER_Z))
mobility_box.SetFixed(True)            # static obstacle, not a free body
sys.AddBody(mobility_box)
sys.GetCollisionSystem().BindAll()     # rebuild collision models after adding the box

# === Driver (constant hard-coded throttle, scripted) ===
# Subclass ChDriver so the 0.8 throttle command is applied every step via the
# standard Synchronize/Advance contract (no human-in-the-loop in headless runs).
class ConstantThrottleDriver(veh.ChDriver):
    def __init__(self, ch_vehicle):
        super().__init__(ch_vehicle)

    def Synchronize(self, time):
        self.SetThrottle(HARDCODED_THROTTLE)   # hard-coded throttle during the loop
        self.SetSteering(STEERING_CMD)
        self.SetBraking(BRAKING_CMD)


driver = ConstantThrottleDriver(tracked)
driver.Initialize()

# === Visualization === full Irrlicht scene: window + sky + camera/chase + lights + grid
if not HEADLESS:
    vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("M113 Tracked Vehicle — Mobility Test")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)   # chase cam (window view)
    vis.Initialize()                                   # Initialize FIRST (Irrlicht order)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()                                    # outdoor sky backdrop
    vis.AddTypicalLights()                             # standard lighting
    vis.AttachVehicle(tracked)                         # bind chassis/track visual assets

# === Main loop (render-cadence outer loop; physics in inner batch) ===
# Terrain forces are gathered per side, sized to the actual track-shoe counts,
# and passed to the 4-arg tracked Synchronize. vehicle.Advance steps the
# wrapper-owned system, so we never call sys.DoStepDynamics here.
shoe_forces_left = veh.TerrainForces(n_shoes_left)     # precomputed buffers, reused each step
shoe_forces_right = veh.TerrainForces(n_shoes_right)

os.makedirs("frames", exist_ok=True)   # guard against missing output dir
os.makedirs("cam", exist_ok=True)      # motion log + review frames live here

data_file = None
motion_file = None
try:
    # Guard the file opens specifically (disk / permission errors).
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:   # disk full / permission denied
        print(f"failed to open output CSV: {exc}")
        raise

    data_writer = csv.writer(data_file)
    data_writer.writerow(["time", "chassis_x", "chassis_y", "chassis_z", "speed", "throttle"])
    motion_writer = csv.writer(motion_file)
    motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    times, xs, zs, speeds = [], [], [], []

    frame = 0
    running = True
    while running and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            if not vis.Run():
                break
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
            frame += 1

        for _ in range(RENDER_EVERY):
            time = sys.GetChTime()
            driver_inputs = driver.GetInputs()

            # Log physics every step.
            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            speed = vel.Length()
            data_writer.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                  f"{pos.z:.5f}", f"{speed:.5f}", f"{HARDCODED_THROTTLE:.3f}"])
            motion_writer.writerow([f"{time:.5f}", "chassis", f"{pos.x:.5f}",
                                    f"{pos.y:.5f}", f"{pos.z:.5f}",
                                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])
            times.append(time)
            xs.append(pos.x)
            zs.append(pos.z)
            speeds.append(speed)

            # Synchronize the subsystem stack (tracked: 4-arg with per-side forces).
            driver.Synchronize(time)
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right)
            if not HEADLESS:
                vis.Synchronize(time, driver_inputs)

            # Advance the subsystem stack. vehicle.Advance steps the owned system.
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)          # advances the wrapper-owned ChSystem
            if not HEADLESS:
                vis.Advance(TIME_STEP)

            if sys.GetChTime() >= RUN_END:
                running = False
                break

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

# === Post-processing (timeseries plot from the logged arrays) ===
try:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.plot(times, xs, label="chassis X (m)", color="tab:blue")
    ax1.plot(times, zs, label="chassis Z (m)", color="tab:green")
    ax1.set_ylabel("position (m)")
    ax1.legend(loc="best")
    ax1.grid(True)
    ax2.plot(times, speeds, label="chassis speed (m/s)", color="tab:red")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("speed (m/s)")
    ax2.legend(loc="best")
    ax2.grid(True)
    fig.suptitle("M113 mobility test — chassis motion under 0.8 throttle")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=120)
    plt.close(fig)
except (OSError, IOError) as exc:   # plotting / file-write failure must not mask sim result
    print(f"failed to write timeseries plot: {exc}")

print(f"done: frames={frame if not HEADLESS else 0} final_time={sys.GetChTime():.3f} "
      f"final_x={xs[-1] if xs else float('nan'):.3f}")
