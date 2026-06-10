"""CityBus on flat rigid terrain — wheeled-vehicle dynamics demo (PyChrono + Irrlicht).

Model
-----
A full catalog CityBus (veh.CityBus wrapper) is initialized at a specified
location/orientation with a TMEASY tire model, then driven across a flat
RigidTerrain patch carrying a custom tile texture. The system is an SMC
(penalty-contact) ChSystemNSC-family system owned by the vehicle wrapper.

Bodies / subsystems (all created inside the veh.CityBus wrapper)
  - chassis rigid body (rendered as a MESH)
  - 4 wheel/spindle bodies + tires (MESH) on 2 axles
  - suspension + steering links (PRIMITIVES visualization)
  - a flat RigidTerrain patch (textured rigid ground body)

Driver
------
A scripted veh.ChDriver subclass supplies steering / throttle / braking as a
function of time (a real headless run gets zero input from ChInteractiveDriver,
so an autonomous scripted driver is used). The bus brakes briefly, then
accelerates straight forward while applying a mild sinusoidal steer.

Expected behavior
-----------------
The chassis accelerates from rest and translates forward (X increases by several
metres over the run) while remaining upright (roll/pitch small, chassis Z roughly
constant). The simulation loop renders/updates at 50 fps. Outputs:
simulation_data.csv (time + chassis pose/speed/inputs), cam/motion_log.csv
(chassis pose + velocity), and simulation_timeseries.png.
"""

# === Imports ===
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

# === Named constants (geometry / physics / cadence) ===
TIME_STEP = 1.0e-3                      # integration step (s)
TIRE_STEP_SIZE = 1.0e-3                 # TMEASY tire internal step (s)
SIM_END = 12.0                          # total simulated time (s)
RENDER_FPS = 50.0                       # review video + render cadence (fps)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once: steps per frame

TERRAIN_LENGTH = 200.0                  # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0                   # rigid patch Y extent (m)
TERRAIN_TOP_Z = 0.0                     # top surface of the flat terrain (m)

# CityBus geometry: chassis origin sits ~0.5 m above the wheel-bottom contact
# plane at rest; tire radius from the catalog wheel geometry.
SUSPENSION_REF_HEIGHT = 0.5             # chassis-origin height above wheel bottom at rest (m)
TIRE_RADIUS = 0.525                     # CityBus catalog tire radius (m), used for the rest-height assert
ZTOL = 0.10                             # allowed wheel-bottom clearance/overlap vs terrain top (m)

INIT_X = -80.0                          # spawn near one end so the bus drives across the patch (m)
INIT_Y = 0.0
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT          # derived rest height (m)

CHASE_TRACKPOINT = chrono.ChVector3d(0.0, 0.0, 1.75)    # point on chassis the camera follows
CHASE_DISTANCE = 14.0                   # camera distance behind the bus (m)
CHASE_HEIGHT = 1.0                      # camera height offset (m)

BRAKE_PHASE_END = 0.5                   # brief brake hold at start (s)
CRUISE_THROTTLE = 0.7                   # throttle after the brake phase
STEER_AMPLITUDE = 0.15                  # mild sinusoidal steering amplitude (-1..1)
STEER_RATE = 0.4                        # steering angular rate (rad/s)

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))    # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END    # short physics check when validating


# === Scripted driver (autonomous; headless-safe) ===
# A ChInteractiveDriver returns zero input headless, so control is scripted as a
# function of time: brake briefly, then cruise straight with a gentle steer.
class ScriptedBusDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < BRAKE_PHASE_END:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(CRUISE_THROTTLE)
            self.SetBraking(0.0)
        self.SetSteering(STEER_AMPLITUDE * math.sin(STEER_RATE * time))


def main():
    # === Vehicle (CityBus wrapper owns the system + bodies + joints) ===
    init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)         # identity: facing +X

    bus = veh.CityBus()
    bus.SetContactMethod(chrono.ChContactMethod_SMC)
    bus.SetChassisCollisionType(veh.CollisionType_NONE)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    bus.SetTireType(veh.TireModelType_TMEASY)           # prompt: a tire model (TMEASY for rigid road)
    bus.SetTireStepSize(TIRE_STEP_SIZE)
    bus.Initialize()

    # Combination of mesh + primitive visualization types for different parts.
    bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.CityBus wrapper) ===
    sys = bus.GetSystem()                       # ChSystem owned by the wrapper
    chassis = bus.GetChassisBody()              # cache: main chassis rigid body, reused every step
    veh_obj = bus.GetVehicle()                  # cache: vehicle subsystem handle, reused for spindle queries
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); 2 axles x 2 sides
    # joints: suspension + steering links created inside the wrapper

    # === Footprint assert (wheels rest on the terrain, not through it) ===
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"bus sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} vs terrain "
        f"top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Terrain (flat rigid patch with a custom texture) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z), chrono.QUNIT),
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver (scripted, autonomous) ===
    driver = ScriptedBusDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    # Gated for fast headless validation; the full vehicle-aware Irrlicht block
    # below is the standard renderer for the on-screen review run.
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("CityBus on Rigid Terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(CHASE_TRACKPOINT, CHASE_DISTANCE, CHASE_HEIGHT)   # camera follows the bus
        vis.Initialize()                                                     # device first
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddLightDirectional()
        vis.AttachVehicle(veh_obj)                                           # binds chassis/wheel/tire assets
        vis.AttachDriver(driver)                                             # steering/throttle/brake HUD bars

    # === Output dirs + CSV writers ===
    os.makedirs("frames", exist_ok=True)        # guard against missing frame output dir
    os.makedirs("cam", exist_ok=True)           # guard against missing motion-log dir

    data_file = None
    motion_file = None
    times, xs, ys, zs, speeds, throttles = [], [], [], [], [], []

    try:
        try:
            data_file = open("simulation_data.csv", "w", newline="")        # disk / permission guard below
            motion_file = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
        except (OSError, IOError) as exc:       # disk full / permission denied
            print(f"failed to open CSV output: {exc}")
            raise

        data_writer = csv.writer(data_file)
        data_writer.writerow(
            ["time", "x", "y", "z", "speed", "throttle", "steering", "braking"]
        )
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow(["time", "x", "y", "z", "vx", "vy", "vz", "speed"])

        # === Main loop === render-cadence outer loop; Synchronize/Advance the
        # full subsystem stack (no sys.DoStepDynamics — Advance already steps it).
        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")     # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # Log physics this step.
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_writer.writerow([
                    f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{speed:.5f}", f"{driver_inputs.m_throttle:.4f}",
                    f"{driver_inputs.m_steering:.4f}", f"{driver_inputs.m_braking:.4f}",
                ])
                motion_writer.writerow([
                    f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}", f"{speed:.5f}",
                ])
                times.append(sim_time)
                xs.append(pos.x); ys.append(pos.y); zs.append(pos.z)
                speeds.append(speed); throttles.append(driver_inputs.m_throttle)

                # Advance subsystem stack (driver/terrain/vehicle[/vis]).
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                bus.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                bus.Advance(TIME_STEP)              # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= RUN_END:
                    break

    except (RuntimeError, ValueError) as exc:       # solver divergence / bad simulation state
        import traceback
        traceback.print_exc()
        print(f"simulation aborted: {exc}")
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === time-series plot from the logged arrays
    if times:
        t = np.array(times)
        fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axes[0].plot(t, xs, label="x")
        axes[0].plot(t, ys, label="y")
        axes[0].plot(t, zs, label="z")
        axes[0].set_ylabel("chassis position (m)")
        axes[0].legend(loc="best"); axes[0].grid(True)
        axes[1].plot(t, speeds, color="tab:red")
        axes[1].set_ylabel("speed (m/s)"); axes[1].grid(True)
        axes[2].plot(t, throttles, color="tab:green")
        axes[2].set_ylabel("throttle"); axes[2].set_xlabel("time (s)"); axes[2].grid(True)
        fig.suptitle("CityBus on rigid terrain — chassis motion")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        print(f"frames written : {0 if HEADLESS else 'see frames/'}")
        print(f"rows logged    : {len(times)}")
        print(f"x: {xs[0]:.3f} -> {xs[-1]:.3f} m (delta {xs[-1] - xs[0]:.3f})")
        print(f"max speed      : {max(speeds):.3f} m/s")


if __name__ == "__main__":
    main()
