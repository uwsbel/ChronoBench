"""CityBus accelerate-and-turn maneuver on flat rigid terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
A full CityBus wheeled-vehicle model (catalog `veh.CityBus` wrapper) drives on a
flat `veh.RigidTerrain` patch. The vehicle wrapper internally creates and owns an
NSC ChSystem, the chassis rigid body, four spindles/wheels, the suspension and
steering joints, the powertrain, and the tire force models.

Control
-------
The vehicle is driven open-loop by a *data-driven* driver (`veh.ChDataDriver`)
built from a fixed table of `veh.DataDriverEntry(time, steering, throttle,
braking, gear)` samples:
  t = 0.0 s : throttle 0.0, steering 0.0, braking 0.0   (standing start)
  t = 0.1 s : throttle 1.0, steering 0.0, braking 0.0   (full acceleration straight)
  t = 0.5 s : throttle 1.0, steering 0.7, braking 0.0   (full acceleration + hard left steer)
The driver linearly interpolates between table rows and holds the final row, so
the bus launches forward, then curves to the left as the steering ramps in.

Expected behavior
-----------------
The bus accelerates from rest, gains forward speed, and once the steering input
becomes active it follows a left-curving path while remaining upright on the
terrain patch (which is sized large enough to contain the swept turn).

System type: NSC (created by the CityBus wrapper). Renderer: Irrlicht.
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

# === Named constants (geometry / physics / control) ===
STEP_SIZE = 1.0e-3                 # integration step (s)
TIRE_STEP_SIZE = 1.0e-3            # tire force-model sub-step (s)
SIM_END = 8.0                      # total simulated time (s)
RENDER_FPS = 30.0                  # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once: physics steps per frame

# Terrain patch sized so the left-curving turn stays fully on-patch.
TERRAIN_LENGTH = 200.0             # X extent (m)
TERRAIN_WIDTH = 200.0             # Y extent (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_YOUNG = 2.0e7             # SMC contact stiffness for the patch material
TERRAIN_HEIGHT = 0.0              # top of the flat patch (world Z)

# CityBus geometric-center chassis origin: spawn height above the patch top.
SUSPENSION_REF_HEIGHT = 0.55      # chassis-origin height above wheel-bottom at rest
VEH_INIT_X = 0.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = TERRAIN_HEIGHT + SUSPENSION_REF_HEIGHT

ZTOL = 0.20                       # allowed wheel-bottom clearance/overlap vs patch top
TIRE_RADIUS_GUESS = 0.5           # nominal CityBus tire radius (m), refined from the tire after Initialize

# Data-driver schedule: DataDriverEntry(time, steering, throttle, braking, gear).
DRIVER_SCHEDULE = [
    (0.0, 0.0, 0.0, 0.0, 0.0),    # standing start
    (0.1, 0.0, 1.0, 0.0, 0.0),    # full throttle, straight
    (0.5, 0.7, 1.0, 0.0, 0.0),    # full throttle, hard left steer
]

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating

OUT_CSV = "simulation_data.csv"
MOTION_CSV = os.path.join("cam", "motion_log.csv")
PLOT_PNG = "simulation_timeseries.png"


def main():
    # === System & bodies (created by the veh.CityBus wrapper) ===
    # The wrapper owns the ChSystemNSC, the chassis, four spindles/wheels, the
    # suspension + steering joints, the powertrain, and the tire models.
    bus = veh.CityBus()
    bus.SetContactMethod(chrono.ChContactMethod_SMC)
    bus.SetChassisCollisionType(veh.CollisionType_NONE)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(
        chrono.ChCoordsysd(
            chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT
        )
    )
    bus.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire so the bus actually drives
    bus.SetTireStepSize(TIRE_STEP_SIZE)
    bus.Initialize()

    # Visualization types (must follow Initialize()).
    bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = bus.GetSystem()                    # cache: ChSystemNSC owned by the wrapper, reused every step
    veh_obj = bus.GetVehicle()                  # cache: ChWheeledVehicle handle, reused for state queries
    chassis = bus.GetChassisBody()              # cache: main chassis rigid body, reused every step

    # === Terrain (flat rigid patch) ===
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(TERRAIN_YOUNG)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_HEIGHT), chrono.QUNIT),
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.Initialize()

    # === Footprint assertion (wheels rest on, not through, the patch) ===
    # Read actual spindle world Z after Initialize; refine the tire radius from
    # the real tire so the wheel-bottom check is accurate (do not trust the guess).
    tire_radius = TIRE_RADIUS_GUESS
    try:
        tire_radius = veh_obj.GetAxles()[0].GetWheels()[0].GetTire().GetRadius()
    except (RuntimeError, AttributeError, IndexError):
        tire_radius = TIRE_RADIUS_GUESS         # fall back to nominal radius if query unavailable

    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - tire_radius
    assert wheel_bottom_z >= TERRAIN_HEIGHT - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs patch top z={TERRAIN_HEIGHT:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_HEIGHT - wheel_bottom_z:.3f} m"
    )

    # === Driver (data-driven, open-loop schedule) ===
    driver_data = veh.vector_Entry(
        [veh.DataDriverEntry(t, s, th, br, g) for (t, s, th, br, g) in DRIVER_SCHEDULE]
    )
    driver = veh.ChDataDriver(veh_obj, driver_data)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("CityBus accelerate-and-turn")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 14.0, 0.6)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(
            2.0, 2.0, 50, 50,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_HEIGHT + 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4),
        )
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Main loop (render-cadence outer, Synchronize/Advance physics inner) ===
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir for review frames
    os.makedirs("cam", exist_ok=True)      # guard against missing output dir for motion log

    sim_f = None
    motion_f = None
    times = []
    speeds = []
    xs = []
    ys = []
    headings = []
    steer_log = []
    throttle_log = []

    try:
        sim_f = open(OUT_CSV, "w", newline="")          # main physics log
        motion_f = open(MOTION_CSV, "w", newline="")    # per-body motion contract log
    except (OSError, IOError) as exc:                    # disk full / permission denied
        print(f"Could not open output CSV: {exc}")
        raise

    try:
        sim_writer = csv.writer(sim_f)
        sim_writer.writerow(
            ["time", "pos_x", "pos_y", "pos_z", "speed",
             "yaw_deg", "steering", "throttle", "braking"]
        )
        motion_writer = csv.writer(motion_f)
        motion_writer.writerow(
            ["time", "body", "pos_x", "pos_y", "pos_z",
             "vel_x", "vel_y", "vel_z", "yaw_deg"]
        )

        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                # Log physics each step.
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                rot = chassis.GetRot()
                yaw = rot.GetCardanAnglesZYX().z   # heading about world Z (rad)
                yaw_deg = math.degrees(yaw)

                sim_writer.writerow(
                    [f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                     f"{speed:.5f}", f"{yaw_deg:.4f}",
                     f"{driver_inputs.m_steering:.4f}",
                     f"{driver_inputs.m_throttle:.4f}",
                     f"{driver_inputs.m_braking:.4f}"]
                )
                motion_writer.writerow(
                    [f"{sim_time:.5f}", "chassis",
                     f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                     f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}", f"{yaw_deg:.4f}"]
                )

                times.append(sim_time)
                speeds.append(speed)
                xs.append(pos.x)
                ys.append(pos.y)
                headings.append(yaw_deg)
                steer_log.append(driver_inputs.m_steering)
                throttle_log.append(driver_inputs.m_throttle)

                # Subsystem synchronize + advance (wrapper.Advance steps the system).
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                bus.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                bus.Advance(STEP_SIZE)          # advances the wrapper-owned ChSystem
                if not HEADLESS:
                    vis.Advance(STEP_SIZE)

                if system.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:    # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close writers even if a step diverges mid-run.
        if sim_f is not None:
            sim_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing (timeseries plot from logged arrays) ===
    if times:
        t = np.array(times)
        fig, axs = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axs[0].plot(t, speeds, label="speed (m/s)")
        axs[0].set_ylabel("speed (m/s)")
        axs[0].legend(loc="best")
        axs[0].grid(True)

        axs[1].plot(t, headings, color="tab:orange", label="yaw (deg)")
        axs[1].set_ylabel("yaw (deg)")
        axs[1].legend(loc="best")
        axs[1].grid(True)

        axs[2].plot(t, steer_log, label="steering")
        axs[2].plot(t, throttle_log, label="throttle")
        axs[2].set_ylabel("driver input")
        axs[2].set_xlabel("time (s)")
        axs[2].legend(loc="best")
        axs[2].grid(True)

        fig.suptitle("CityBus accelerate-and-turn (data-driven)")
        fig.tight_layout()
        with open(PLOT_PNG, "wb") as pf:   # ensure the file handle closes/flushes
            fig.savefig(pf)
        plt.close(fig)

        # Quick physics summary for the run log.
        print(f"steps logged: {len(times)}")
        print(f"final pos = ({xs[-1]:.2f}, {ys[-1]:.2f})")
        print(f"max speed = {max(speeds):.2f} m/s, final yaw = {headings[-1]:.1f} deg")
        print(f"net XY displacement = "
              f"{math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]):.2f} m")


if __name__ == "__main__":
    main()
