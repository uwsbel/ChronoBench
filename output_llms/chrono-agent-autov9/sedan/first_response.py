"""BMW E90 Sedan driving on rigid terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
A wrapper-managed BMW E90 wheeled vehicle (``veh.Sedan``) is placed on a flat
``veh.RigidTerrain`` patch and driven by a scripted ``veh.ChDriver`` subclass
that ramps throttle and applies a sinusoidal steering signal (the autonomous,
headless-safe equivalent of the interactive steering/throttle/braking control
the scene calls for). The vehicle uses a TMEASY handling tire model and a
configurable chassis collision / visualization setup.

System type
-----------
NSC (non-smooth contact). The ``veh.Sedan`` wrapper creates and OWNS its
``ChSystemNSC``; terrain, driver, and the Irrlicht chase-camera visualization are
all attached to that single owned system. We never create a second ``ChSystem``.

Main bodies
-----------
- chassis rigid body (BMW E90 sprung mass), fetched via ``GetChassisBody``
- four wheel spindles (two axles), created inside the wrapper
- a fixed rigid terrain patch body (textured, with a logo decal)

Expected behavior
-----------------
The four wheels rest on the terrain at t=0 (asserted from the spindle world
positions). As throttle ramps in, the sedan accelerates forward along +X while
the sinusoidal steering produces a gentle weave; the chassis stays upright. The
forward displacement, speed, and chassis height are logged and plotted.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless PNG backend — no display needed for the plot
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Named constants (geometry / physics / control) ===
TIME_STEP = 2.0e-3                 # integration step (s)
SIM_END = 12.0                     # total simulated time (s)
RENDER_FPS = 30.0                  # review-video frame rate

# Vehicle placement. Sedan chassis origin is the GEOMETRIC CENTER (axles at
# +/-1.388 m in X, symmetric), so the spawn X/Y is the body center directly.
VEH_INIT_X = 0.0
VEH_INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.20       # chassis-origin height above wheel-bottom at rest
TERRAIN_TOP_Z = 0.0                # flat rigid-terrain surface height
VEH_INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT
TIRE_RADIUS = 0.3266               # BMW E90 tire radius (from wheel geometry)
ZTOL = 0.05                        # allowed wheel-bottom clearance/overlap vs terrain

# Terrain patch (flat, paved-road baseline).
TERRAIN_LENGTH = 300.0
TERRAIN_WIDTH = 60.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# Scripted control law timing.
THROTTLE_RAMP_END = 3.0            # seconds to reach cruise throttle
CRUISE_THROTTLE = 0.6
STEER_AMPLITUDE = 0.20             # +/- steering (sinusoidal weave)
STEER_OMEGA = 0.5                  # rad/s of the steering sinusoid

# Headless validation gate — a short, windowless physics check (fast).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END

# Derived constants — precomputed once, never recomputed in the loop.
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === Scripted driver (autonomous, headless-safe steering/throttle/braking) ===
class SedanDriver(veh.ChDriver):
    """Time-based control law: ramp throttle, hold cruise, sinusoidal steer.

    A scripted ChDriver subclass replaces a keyboard ChInteractiveDriver so the
    control runs identically in a windowless validation pass and in the rendered
    run (an interactive driver would read zero input headless).
    """

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < THROTTLE_RAMP_END:
            self.SetThrottle(CRUISE_THROTTLE * (time / THROTTLE_RAMP_END))
        else:
            self.SetThrottle(CRUISE_THROTTLE)
        self.SetBraking(0.0)
        self.SetSteering(STEER_AMPLITUDE * math.sin(STEER_OMEGA * time))


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing frame output dir
    os.makedirs("cam", exist_ok=True)       # review video + motion log live here

    # === Vehicle (wrapper creates & owns the ChSystemNSC + bodies) ===
    # Build the BMW E90 Sedan, choose contact/tire models, then Initialize().
    vehicle = veh.Sedan()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)   # configurable collision
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z),
                           chrono.QUNIT)
    )
    vehicle.SetTireType(veh.TireModelType_TMEASY)   # prompt: TMEASY handling tire
    vehicle.SetTireStepSize(TIME_STEP)
    vehicle.Initialize()

    # Visualization detail levels (configurable per the scene description).
    vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.Sedan wrapper) ===
    sys = vehicle.GetSystem()                 # ChSystemNSC owned by the wrapper
    chassis = vehicle.GetChassisBody()        # cache: main chassis body, reused every step
    veh_obj = vehicle.GetVehicle()            # cache: ChWheeledVehicle, reused every step
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side) — two axles, LEFT/RIGHT
    # joints: suspension + steering links are created inside the wrapper

    # === Terrain (flat rigid patch on the wrapper-owned system) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    # Customizable texture + logo decal on the terrain surface.
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.85))
    terrain.Initialize()

    # === Footprint assert (wheels rest on the terrain, not through it) ===
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

    # === Driver (scripted, attached to the vehicle) ===
    driver = SedanDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht chase-camera scene (window + sky + camera + lights + grid)
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("BMW E90 Sedan on Rigid Terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)  # behind/above chassis
        vis.Initialize()                                   # Initialize FIRST (Irrlicht)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                    # outdoor sky backdrop
        vis.AddTypicalLights()                             # standard directional lighting
        vis.AddLight(chrono.ChVector3d(30.0, 30.0, 60.0), 200.0, chrono.ChColor(0.9, 0.9, 0.9))
        vis.AddGrid(2.0, 2.0, 60, 30,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))         # ground reference grid
        vis.AttachVehicle(veh_obj)                         # bind chassis/wheel/tire assets
        vis.AttachDriver(driver)                           # HUD throttle/steer/brake bars

    # === Main loop (render-cadence outer loop; Synchronize/Advance per step) ===
    data_file = None
    motion_file = None
    times, speeds, pos_x, pos_z = [], [], [], []
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(["time", "pos_x", "pos_y", "pos_z", "speed",
                              "throttle", "steering"])
        motion_writer.writerow(["time", "body", "x", "y", "z",
                                "vx", "vy", "vz"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # --- log physics each step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_writer.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                      f"{pos.z:.5f}", f"{speed:.5f}",
                                      f"{driver_inputs.m_throttle:.4f}",
                                      f"{driver_inputs.m_steering:.4f}"])
                motion_writer.writerow([f"{time:.5f}", "chassis",
                                        f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                        f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])
                times.append(time)
                speeds.append(speed)
                pos_x.append(pos.x)
                pos_z.append(pos.z)

                # Synchronize the full subsystem stack, then advance it.
                driver.Synchronize(time)
                terrain.Synchronize(time)
                vehicle.Synchronize(time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                vehicle.Advance(TIME_STEP)   # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:           # disk / permission while writing CSV
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverges mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing (timeseries plot from the logged arrays) ===
    t = np.array(times)
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    axes[0].plot(t, np.array(speeds), color="tab:blue")
    axes[0].set_ylabel("speed (m/s)")
    axes[0].grid(True)
    axes[1].plot(t, np.array(pos_x), color="tab:green")
    axes[1].set_ylabel("x position (m)")
    axes[1].grid(True)
    axes[2].plot(t, np.array(pos_z), color="tab:red")
    axes[2].set_ylabel("z position (m)")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(True)
    fig.suptitle("BMW E90 Sedan on Rigid Terrain")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

    print(f"done: simulated {t[-1]:.3f} s, final x={pos_x[-1]:.2f} m, "
          f"final speed={speeds[-1]:.2f} m/s, frames={'(headless)' if HEADLESS else frame}")


if __name__ == "__main__":
    main()
