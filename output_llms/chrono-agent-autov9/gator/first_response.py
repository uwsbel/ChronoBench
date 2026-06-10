"""Gator wheeled vehicle on flat rigid terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
A `veh.Gator` light off-road utility vehicle is created with an NSC contact
method and a TMEASY tire model, then driven across a flat `veh.RigidTerrain`
patch that carries a custom dirt texture. The Gator wrapper owns its own
`ChSystemNSC`; the terrain, driver, and the vehicle-aware Irrlicht visualizer
are all attached to that single owned system.

System type
-----------
NSC (non-smooth contact). All vehicle components (chassis, wheels, tires,
suspension, steering) use mesh visualization.

Main bodies
-----------
- chassis: the Gator chassis rigid body (wrapper-created)
- four wheel spindles / tires (two axles), suspension + steering links
- terrain: a single flat RigidTerrain patch (the support plane at z = 0)

Driver / control
----------------
A scripted `veh.ChDriver` subclass supplies steering, throttle, and braking as
a time-based control law (the batch/headless analogue of an interactive driver:
brief settle, then accelerate forward with a gentle steering sweep). Inputs are
read through `GetInputs()` and applied via the Synchronize/Advance contract.

Expected behavior
-----------------
The Gator rests with all four wheels on the terrain, then accelerates forward
(monotonically increasing X position and forward speed) while steering gently,
staying upright. The render-cadence loop targets a 50 fps review video in
real time.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for the post-run plot
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / control) ===
TIME_STEP = 1.0e-3                 # integrator step (s)
SIM_END = 10.0                     # total simulated time (s)
RENDER_FPS = 50.0                  # review video frame rate (fps)

TERRAIN_LENGTH = 200.0             # rigid terrain patch X size (m)
TERRAIN_WIDTH = 100.0              # rigid terrain patch Y size (m)
TERRAIN_TOP_Z = 0.0                # terrain top surface height (m)

TIRE_RADIUS = 0.28575              # Gator tire radius (m) — from wheel geometry
SUSPENSION_REF_HEIGHT = 0.32       # chassis-origin height above wheel-bottom at rest (m)
INIT_X = 0.0                       # spawn X (m)
INIT_Y = 0.0                       # spawn Y (m)
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT  # derived chassis-origin Z (m)
ZTOL = 0.05                        # allowed wheel-bottom clearance/overlap vs terrain (m)

FRICTION = 0.9                     # terrain friction coefficient
RESTITUTION = 0.01                 # terrain restitution

SETTLE_TIME = 0.5                  # s of zero throttle to let the vehicle settle
DRIVE_THROTTLE = 0.6               # cruise throttle after settle (0..1)
STEER_AMPLITUDE = 0.25             # peak steering command (-1..1)
STEER_RATE = 0.4                   # steering sweep angular rate (rad/s)

# Derived render cadence — precomputed once, never recomputed in the loop.
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Fast, windowless validation run (short bounded physics check).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short check when validating


# === Driver: scripted time-based control (headless analogue of interactive) ===
class GatorDriver(veh.ChDriver):
    """Scripted driver: settle, then accelerate forward with a steering sweep."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < SETTLE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)   # hold still while the suspension settles
            self.SetSteering(0.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
            # gentle sinusoidal steering sweep
            self.SetSteering(STEER_AMPLITUDE * math.sin(STEER_RATE * (time - SETTLE_TIME)))


def main():
    # === Vehicle (wrapper builds and owns the system + bodies + joints) ===
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)      # NSC system
    gator.SetChassisCollisionType(veh.CollisionType_NONE)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT))
    gator.SetTireType(veh.TireModelType_TMEASY)             # prompt: TMEASY tire model
    gator.SetTireStepSize(TIME_STEP)
    gator.Initialize()

    # Mesh visualization for all vehicle components (prompt requirement).
    gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
    gator.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
    gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    gator.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.Gator wrapper) ===
    sys = gator.GetSystem()                 # ChSystemNSC owned by the wrapper
    chassis = gator.GetChassisBody()        # cache: main chassis rigid body, reused every step
    veh_obj = gator.GetVehicle()            # cache: ChWheeledVehicle handle, reused every step
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); suspension + steering links
    # are created inside the wrapper; terrain patch body is added below.

    # === Footprint assert — wheels must rest on (not through) the terrain ===
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )
    assert wheel_bottom_z <= TERRAIN_TOP_Z + 0.2, (
        f"vehicle floats above terrain: wheel bottom z={wheel_bottom_z:.3f}; "
        f"lower SUSPENSION_REF_HEIGHT"
    )

    # === Terrain (flat rigid patch on the wrapper-owned system) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(FRICTION)
    patch_mat.SetRestitution(RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 200, 200)  # custom texture
    patch.SetColor(chrono.ChColor(0.7, 0.6, 0.45))
    terrain.Initialize()

    # === Driver (scripted) ===
    driver = GatorDriver(veh_obj)
    driver.Initialize()

    # === Visualization (vehicle-aware Irrlicht; full scene built inline) ===
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("Gator on Rigid Terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.0), 8.0, 0.5)  # follow camera
        vis.Initialize()                                                # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                                 # sky backdrop
        vis.AddTypicalLights()                                          # standard lighting
        vis.AddCamera(chrono.ChVector3d(-8.0, -8.0, 4.0),
                      chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z))        # static overview eye
        vis.AddGrid(2.0, 2.0, 50, 50,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))                      # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)                                        # steering/throttle/brake HUD

    # === Output directories / CSV writers ===
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
    os.makedirs("cam", exist_ok=True)      # guard against missing motion-log dir

    try:
        data_f = open("simulation_data.csv", "w", newline="")   # primary physics log
        motion_f = open("cam/motion_log.csv", "w", newline="")  # per-body motion contract log
    except (OSError, IOError) as exc:                            # disk / permission failure
        print(f"Could not open output CSV: {exc}")
        raise

    data_w = csv.writer(data_f)
    motion_w = csv.writer(motion_f)
    data_w.writerow(["time", "x", "y", "z", "speed", "throttle", "steering", "braking"])
    motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    # Time series for the post-run plot.
    t_hist, x_hist, speed_hist, throttle_hist = [], [], [], []

    # === Main loop (render-cadence outer; Synchronize/Advance inner) ===
    frame = 0
    try:
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # log physics every step
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_w.writerow([
                    f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{speed:.5f}", f"{driver_inputs.m_throttle:.4f}",
                    f"{driver_inputs.m_steering:.4f}", f"{driver_inputs.m_braking:.4f}",
                ])
                motion_w.writerow([
                    f"{sim_time:.5f}", "chassis",
                    f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
                ])
                t_hist.append(sim_time)
                x_hist.append(pos.x)
                speed_hist.append(speed)
                throttle_hist.append(driver_inputs.m_throttle)

                # Synchronize the full subsystem stack, then advance it.
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                gator.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                gator.Advance(TIME_STEP)        # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush partial output even if a step diverges.
        data_f.close()
        motion_f.close()

    # === Post-processing (time-series plot) ===
    t = np.array(t_hist)
    fig, ax1 = plt.subplots(figsize=(9, 5))
    ax1.plot(t, np.array(x_hist), "b-", label="x position (m)")
    ax1.plot(t, np.array(speed_hist), "g-", label="speed (m/s)")
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("position / speed")
    ax2 = ax1.twinx()
    ax2.plot(t, np.array(throttle_hist), "r--", label="throttle")
    ax2.set_ylabel("throttle (0..1)")
    ax1.legend(loc="upper left")
    ax2.legend(loc="upper right")
    ax1.set_title("Gator on rigid terrain — forward motion")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

    print(f"Done: frames={frame}, steps={len(t_hist)}, "
          f"final_x={x_hist[-1]:.3f} m, final_speed={speed_hist[-1]:.3f} m/s")


if __name__ == "__main__":
    main()
