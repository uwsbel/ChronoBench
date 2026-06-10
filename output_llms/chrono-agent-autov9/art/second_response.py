"""ARTcar small-scale RC vehicle driving on a flat rigid terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
- System: a single ChSystemNSC owned internally by the veh.ARTcar wrapper
  (chassis + 4 spindles + suspension/steering links are created by the wrapper).
- Terrain: a flat veh.RigidTerrain patch (Bullet contacts) under the vehicle.
- Vehicle: veh.ARTcar (1/6-scale RC car). Spawned at world (1, 0, 0.5).
    * Vehicle-part visualization: PRIMITIVES (boxes/cylinders, not mesh).
    * Chassis collision: CollisionType_MESH (chassis collides with the world).
    * Tire model: TireModelType_FIALA (analytical slip-based tire force model).
- Driver: a scripted veh.ChDriver subclass — brief brake-to-settle, then a
  steady throttle with a gentle sinusoidal steering sweep.

Expected behavior
------------------
After a short settling phase the ARTcar should accelerate forward from x=1 and
travel several meters while staying upright (chassis Z roughly constant, small
roll/pitch). The CSV logs chassis pose/speed for the physics check; review
frames are rendered from the Irrlicht chase camera.

Note: FIALA tires (like TMEASY) get ground contact through the terrain object,
so vehicle.Synchronize(time, driver_inputs, terrain) MUST be called with the
terrain argument every step — otherwise the wheels have no support and the car
falls through the patch.
"""

import os
import csv
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# === Named constants === geometry / physics / control parameters (no bare literals downstream)
TIME_STEP = 1.0e-3                # integration step (s)
TIRE_STEP = 1.0e-3               # tire substep (s) — required for non-rigid (FIALA) tires
SIM_END = 8.0                    # simulation duration (s)
RENDER_FPS = 50.0                # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once: steps per frame

TERRAIN_LENGTH = 160.0           # rigid patch X extent (m) — keeps the car on the patch the whole run
TERRAIN_WIDTH = 160.0            # rigid patch Y extent (m)
TERRAIN_FRICTION = 0.9           # tire/ground friction
TERRAIN_RESTITUTION = 0.01       # ground bounciness

INIT_X = 1.0                     # vehicle spawn X (world)
INIT_Y = 0.0                     # vehicle spawn Y (world)
INIT_Z = 0.5                     # vehicle spawn Z (world) — chassis origin above the patch
INIT_LOC = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.QUNIT          # facing +X, level

SETTLE_TIME = 0.5                # brake-only settle phase (s)
DRIVE_THROTTLE = 0.6             # steady throttle after settle (0..1)
STEER_AMPL = 0.3                 # peak steering (−1..+1)
STEER_FREQ = 0.4                 # steering sweep frequency (Hz)

CHASE_TRACK = chrono.ChVector3d(0.0, 0.0, 0.2)   # chase-cam track point on chassis
CHASE_DIST = 3.0                 # camera distance behind vehicle (m) — close for RC scale
CHASE_HEIGHT = 0.6               # camera height offset (m)

OUT_CSV = "simulation_data.csv"
MOTION_CSV = "cam/motion_log.csv"
PLOT_PNG = "simulation_timeseries.png"

# Fast, windowless validation run (short bounded sim, no Irrlicht window).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating


# === Driver === scripted time-based ChDriver subclass (no human-in-the-loop)
class ScriptedDriver(veh.ChDriver):
    """Brakes briefly to settle, then applies steady throttle + sinusoidal steering."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < SETTLE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
            self.SetSteering(STEER_AMPL * math.sin(2.0 * math.pi * STEER_FREQ * time))


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
    os.makedirs("cam", exist_ok=True)       # guard against missing motion-log dir

    # === Vehicle === build the ARTcar wrapper, set requested options, then Initialize
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisFixed(False)
    car.SetChassisCollisionType(veh.CollisionType_MESH)        # prompt: chassis collision = MESH
    car.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    car.SetTireType(veh.TireModelType_FIALA)                   # prompt: tire model = FIALA
    car.SetTireStepSize(TIRE_STEP)                              # required for non-rigid tires
    car.Initialize()

    # Vehicle-part visualization = PRIMITIVES (note: VisualizationType_* lives in chrono.*)
    car.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

    # === System & bodies (created by the veh.ARTcar wrapper) ===
    sys = car.GetSystem()                 # cache: ChSystemNSC owned by the wrapper, reused every step
    veh_obj = car.GetVehicle()            # cache: vehicle subsystem handle, reused for spindle queries
    chassis = car.GetChassisBody()        # cache: main chassis rigid body, reused every step
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); links: suspension + steering created internally.

    # === Terrain === flat rigid patch providing ground contact for the FIALA tires
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Footprint sanity: with FIALA tires the wheel bottoms must rest on the patch (z=0).
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    min_spindle_z = min(p.z for p in spindle_world)
    assert min_spindle_z > 0.0, (
        f"spindles initialized below the patch (min z={min_spindle_z:.3f}); raise INIT_Z"
    )

    # === Driver === scripted controller bound to the vehicle
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full vehicle-aware Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("ARTcar on Rigid Terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(CHASE_TRACK, CHASE_DIST, CHASE_HEIGHT)
        vis.Initialize()                                                  # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # AFTER Initialize
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(INIT_X - 3.0, INIT_Y - 3.0, INIT_Z + 1.5), INIT_LOC)
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 80, 80,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Main loop === render-cadence outer loop; advance the full subsystem stack inline
    data_f = None
    motion_f = None
    times, speeds, xs, zs = [], [], [], []   # collected for the post-run plot
    try:
        data_f = open(OUT_CSV, "w", newline="")        # guarded by finally below
        motion_f = open(MOTION_CSV, "w", newline="")
        data_w = csv.writer(data_f)
        motion_w = csv.writer(motion_f)
        data_w.writerow(["time", "x", "y", "z", "speed", "throttle", "steering"])
        motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # log this step
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_w.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                 f"{pos.z:.5f}", f"{speed:.5f}",
                                 f"{driver_inputs.m_throttle:.4f}",
                                 f"{driver_inputs.m_steering:.4f}"])
                motion_w.writerow([f"{time:.5f}", "chassis", f"{pos.x:.5f}",
                                   f"{pos.y:.5f}", f"{pos.z:.5f}",
                                   f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])
                times.append(time); speeds.append(speed); xs.append(pos.x); zs.append(pos.z)

                # synchronize + advance the full subsystem stack (terrain arg is mandatory)
                driver.Synchronize(time)
                terrain.Synchronize(time)
                car.Synchronize(time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                car.Advance(TIME_STEP)          # advances the wrapper-owned ChSystem
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= RUN_END:
                    break
    except (OSError, IOError) as exc:           # disk / permission errors on CSV I/O
        import traceback; traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
        import traceback; traceback.print_exc()
        raise
    finally:
        # flush + close any open writers even if a step diverged mid-run
        if data_f is not None:
            data_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing === plot logged channels vs time
    if times:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax1.plot(times, speeds, label="speed (m/s)")
        ax1.set_ylabel("speed (m/s)"); ax1.grid(True); ax1.legend()
        ax2.plot(times, xs, label="chassis x (m)")
        ax2.plot(times, zs, label="chassis z (m)")
        ax2.set_xlabel("time (s)"); ax2.set_ylabel("position (m)")
        ax2.grid(True); ax2.legend()
        fig.suptitle("ARTcar — speed and chassis position")
        fig.tight_layout()
        with open(PLOT_PNG, "wb") as pf:   # context-managed file for the PNG
            fig.savefig(pf)
        plt.close(fig)

    print(f"done: {len(times)} steps logged, final x={xs[-1]:.3f} m" if times else "no steps logged")


if __name__ == "__main__":
    main()
