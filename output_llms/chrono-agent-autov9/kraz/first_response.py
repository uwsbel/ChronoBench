"""Kraz tractor-semitrailer on flat rigid terrain (PyChrono 9.0.1 + Irrlicht).

Model
-----
A Kraz heavy-duty truck (tractor + semitrailer) driven by a scripted open-loop
driver across a flat rigid terrain patch. The vehicle wrapper (`veh.Kraz`)
creates and owns an NSC `ChSystem`; it builds the tractor chassis, the trailer,
all suspension/steering links, wheels and tires internally. A `veh.RigidTerrain`
patch with defined friction and restitution provides the ground. A
`veh.ChWheeledVehicleVisualSystemIrrlicht` window renders the scene with a chase
camera, sky box, lights and a ground grid.

System type
-----------
NSC (non-smooth contact), owned by the Kraz wrapper. Gravity -Z (Z-up world).

Main bodies
-----------
- Tractor chassis (rigid body, the camera chase target)
- Trailer chassis + axles, all wheels/tires (created by the wrapper)
- Rigid terrain patch (flat ground)

Expected behavior
-----------------
The truck starts at rest with wheels resting on the terrain. The scripted driver
releases the brake after a short settle, applies throttle and a gentle steering
sweep, so the tractor accelerates forward and the chassis translates a few meters
while remaining upright. CSV logs confirm forward motion and a near-upright pose.
"""

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

# === Named constants (geometry / physics) ===
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire substep (s)
SIM_END = 10.0                     # total simulated time (s)
RENDER_FPS = 30.0                  # review-video frame rate

TERRAIN_LENGTH = 200.0             # X extent of rigid patch (m)
TERRAIN_WIDTH = 60.0               # Y extent of rigid patch (m)
TERRAIN_TOP_Z = 0.0                # top surface height of the flat patch (m)
TERRAIN_FRICTION = 0.9             # patch friction coefficient
TERRAIN_RESTITUTION = 0.01         # patch restitution (bounciness)

TIRE_RADIUS = 0.5588               # Kraz tire radius (m), from wheel geometry
WHEEL_CLEARANCE = 0.02             # small gap so wheels start just above ground
# init z is the spindle/axle reference plane height (verified: spindle z == init z)
VEH_INIT_X = -90.0                 # spawn near one end so the truck can drive forward
VEH_INIT_Y = 0.0
VEH_INIT_Z = TERRAIN_TOP_Z + TIRE_RADIUS + WHEEL_CLEARANCE

SETTLE_TIME = 0.5                  # hold brake while suspension settles (s)
DRIVE_THROTTLE = 0.7               # throttle once moving
STEER_AMPLITUDE = 0.15             # gentle steering sweep amplitude (-1..1)
STEER_PERIOD = 8.0                 # steering sweep period (s)

ZTOL = 0.06                        # tolerance for wheel-bottom-on-terrain assert
UPRIGHT_MIN = 0.7                  # min chassis up-axis Z to be considered upright

# === Derived constants (precomputed once) ===
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # physics steps per frame
init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
init_rot = chrono.QUNIT


# === Driver (scripted open-loop control law) ===
class ScriptedDriver(veh.ChDriver):
    """Brake-then-drive driver with a gentle sinusoidal steering sweep."""

    def __init__(self, tractor_vehicle):
        super().__init__(tractor_vehicle)

    def Synchronize(self, time):
        if time < SETTLE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
        self.SetSteering(STEER_AMPLITUDE * math.sin(2.0 * math.pi * time / STEER_PERIOD))


# === Headless validation gate ===
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

os.makedirs("frames", exist_ok=True)   # guard against missing frame output dir
os.makedirs("cam", exist_ok=True)       # guard against missing cam output dir

data_file = None
motion_file = None
data_writer = None
motion_writer = None

try:
    # === System & bodies (created by the veh.Kraz wrapper) ===
    vehicle = veh.Kraz()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC truck + rigid terrain
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    vehicle.SetTireStepSize(TIRE_STEP)
    vehicle.Initialize()

    # Kraz visualization setters take tractor + trailer args (steering takes one).
    vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH,
                                        chrono.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES,
                                           chrono.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH,
                                      chrono.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH,
                                     chrono.VisualizationType_MESH)

    sys = vehicle.GetSystem()                       # ChSystem (NSC) owned by the wrapper
    tractor = vehicle.GetTractor()                  # cache: tractor ChWheeledVehicle, reused every step
    chassis_body = vehicle.GetTractorChassisBody()  # cache: tractor chassis rigid body, reused every step
    # trailer: vehicle.GetTrailer(); axles/wheels/tires + suspension & steering links
    # are all created inside the wrapper and ride on the same owned ChSystem.

    # === Terrain (flat rigid patch with defined friction / restitution) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z), chrono.QUNIT),
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.Initialize()

    # --- Assert wheels start on (not through) the terrain ---
    spindle_z = [
        tractor.GetSpindlePos(a, side).z
        for a in range(tractor.GetNumberAxles())
        for side in (veh.LEFT, veh.RIGHT)
    ]
    wheel_bottom_z = min(spindle_z) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise VEH_INIT_Z"
    )

    # === Driver (scripted) ===
    driver = ScriptedDriver(tractor)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Kraz tractor-semitrailer on rigid terrain")
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 14.0, 0.6)   # follow tractor chassis
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 12, -12, 6),
                      chrono.ChVector3d(VEH_INIT_X, 0, 1))
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 60, 30,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z + 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))
        vis.AttachVehicle(vehicle.GetTractor())
        vis.AttachDriver(driver)

    # === Main loop === render-cadence outer loop; Synchronize/Advance the subsystem stack
    with open("simulation_data.csv", "w", newline="") as data_file, \
         open("cam/motion_log.csv", "w", newline="") as motion_file:
        data_writer = csv.writer(data_file)
        data_writer.writerow(["time", "speed", "chassis_x", "chassis_y", "chassis_z",
                              "throttle", "steering", "up_z"])
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow(["time", "body", "x", "y", "z",
                                "vx", "vy", "vz", "up_z"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(render_every):
                sim_time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # --- log physics each step ---
                pos = chassis_body.GetPos()
                vel = chassis_body.GetPosDt()
                up_z = chassis_body.GetRot().GetAxisZ().z   # chassis up-axis Z component
                speed = tractor.GetSpeed()
                data_writer.writerow([
                    f"{sim_time:.5f}", f"{speed:.5f}",
                    f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{driver_inputs.m_throttle:.4f}", f"{driver_inputs.m_steering:.4f}",
                    f"{up_z:.5f}",
                ])
                motion_writer.writerow([
                    f"{sim_time:.5f}", "tractor_chassis",
                    f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}", f"{up_z:.5f}",
                ])

                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                vehicle.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                vehicle.Advance(TIME_STEP)        # advances the wrapper-owned ChSystem
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= run_end:
                    break

except (RuntimeError, ValueError) as exc:           # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:                   # disk / permission on CSV or frames
    import traceback
    traceback.print_exc()
    raise
finally:
    # flush is handled by the `with open(...)` context managers above; nothing left open here.
    pass

# === Post-processing === plot logged time series from the CSV
try:
    with open("simulation_data.csv", "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [r for r in reader if r]
except (OSError, IOError):                          # plotting is best-effort; skip if unreadable
    rows = []

if rows:
    data = np.array([[float(v) for v in r] for r in rows])
    t = data[:, 0]
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    axes[0].plot(t, data[:, 1], label="speed (m/s)")
    axes[0].set_ylabel("speed (m/s)"); axes[0].grid(True); axes[0].legend()
    axes[1].plot(t, data[:, 2], label="chassis x")
    axes[1].plot(t, data[:, 3], label="chassis y")
    axes[1].plot(t, data[:, 4], label="chassis z")
    axes[1].set_ylabel("position (m)"); axes[1].grid(True); axes[1].legend()
    axes[2].plot(t, data[:, 5], label="throttle")
    axes[2].plot(t, data[:, 6], label="steering")
    axes[2].set_ylabel("driver input"); axes[2].set_xlabel("time (s)")
    axes[2].grid(True); axes[2].legend()
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

    # --- physics sanity prints ---
    moved = float(data[-1, 2] - data[0, 2])
    print(f"frames written, rows={len(rows)}, forward displacement={moved:.3f} m, "
          f"final speed={data[-1, 1]:.3f} m/s, min up_z={data[:, 7].min():.3f}")
