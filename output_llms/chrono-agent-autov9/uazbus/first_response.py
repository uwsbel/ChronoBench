"""UAZBUS wheeled vehicle on flat rigid terrain — driven, real-time Irrlicht scene.

Model
-----
A UAZBUS off-road wheeled vehicle (PyChrono ``veh.UAZBUS`` wrapper) initialized
at a known pose and driven by a scripted driver over a flat ``veh.RigidTerrain``
patch with specified friction and restitution.

System type
-----------
NSC (non-smooth contact). The vehicle wrapper creates and owns its own
``ChSystemNSC``; the terrain, driver, and visualization all attach to that one
owned system. The wrapper internally builds the chassis rigid body, four wheel
spindles, the suspension/steering joints, the engine + transmission, and the
tire force models.

Main bodies
-----------
- chassis rigid body (the bus body)
- four wheel spindles (two axles, left/right) with TMEASY tires
- one fixed rigid-terrain patch body (the ground)

Expected behavior
------------------
After a brief launch the bus accelerates forward under throttle, executes one
gentle net-zero left-then-right steer pulse, and otherwise drives straight,
staying upright with all four wheels riding on the terrain and well inside the
patch. The chassis forward displacement grows monotonically while the chassis Z
stays near its rest height — logged to CSV and plotted.
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

# === Named constants — geometry / physics / run control ===
TIME_STEP = 1.0e-3              # integration step (s)
SIM_END = 12.0                 # total simulated time (s)
RENDER_FPS = 50.0              # review-video frame rate

# Vehicle spawn pose. UAZBUS chassis origin sits at the front axle; the wheel
# bottom rest height equals the chassis-origin Z minus the tire radius, so we
# seed the chassis Z at the tire radius (+ small clearance) to drop the wheels
# onto the z=0 terrain plane.
TIRE_RADIUS = 0.372           # UAZBUS tire radius (m), confirmed from wheel geometry
SPAWN_CLEARANCE = 0.02        # initial wheel-bottom clearance above terrain (m)
INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = TIRE_RADIUS + SPAWN_CLEARANCE   # derived chassis-origin height
ZTOL = 0.10                   # allowed wheel-bottom deviation vs terrain top

# Terrain (flat rigid patch) parameters. Sized generously in both directions so
# the bus stays well inside the patch for the whole run even with a steer pulse.
TERRAIN_LENGTH = 400.0        # X extent (m) — long enough for forward driving
TERRAIN_WIDTH = 400.0         # Y extent (m)
TERRAIN_FRICTION = 0.85       # patch friction coefficient
TERRAIN_RESTITUTION = 0.01    # patch restitution (near-inelastic ground)
TERRAIN_TOP_Z = 0.0           # terrain surface height

# Scripted driver schedule. The bus launches, cruises mostly straight, and
# executes one gentle left-then-right steer pulse whose heading change cancels
# out (net-zero), so it returns to a straight forward heading and stays on the
# patch instead of curving off the edge.
LAUNCH_TIME = 1.0             # hold still then accelerate after this (s)
CRUISE_THROTTLE = 0.6         # steady throttle once launched
STEER_AMPLITUDE = 0.18        # gentle steer-pulse amplitude (-1..1)
STEER_START = 3.0             # pulse begins (s)
STEER_PERIOD = 4.0            # one full left-then-right cycle (s) -> net-zero heading

# Headless validation gate: a fast, windowless physics check (no Irrlicht
# window, short bounded run) used only to verify the script runs clean.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short check when validating

# Derived render cadence — precomputed once (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


# === Driver — scripted time-based control (ChDriver subclass) ===
# Open-loop launch-then-cruise with a gentle sinusoidal steer. A scripted driver
# (not the keyboard ChInteractiveDriver) is mandatory for a headless batch run.
class ScriptedDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < LAUNCH_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(CRUISE_THROTTLE)
            self.SetBraking(0.0)
        # One net-zero left-then-right steer pulse, otherwise drive straight.
        if STEER_START <= time < STEER_START + STEER_PERIOD:
            phase = (time - STEER_START) / STEER_PERIOD       # 0..1 over the pulse
            self.SetSteering(STEER_AMPLITUDE * math.sin(2.0 * math.pi * phase))
        else:
            self.SetSteering(0.0)


def build_scene():
    """Build the UAZBUS, rigid terrain, driver, and return named handles."""
    # === Vehicle wrapper (owns the system, chassis, spindles, joints, tires) ===
    bus = veh.UAZBUS()
    bus.SetContactMethod(chrono.ChContactMethod_NSC)        # NSC system
    bus.SetChassisCollisionType(veh.CollisionType_NONE)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    bus.SetTireType(veh.TireModelType_TMEASY)               # slip/grip tire on rigid road
    bus.SetTireStepSize(TIME_STEP)
    bus.Initialize()

    # Visualization detail (must follow Initialize()).
    bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.UAZBUS wrapper) ===
    system = bus.GetSystem()                 # ChSystemNSC owned by the wrapper
    veh_obj = bus.GetVehicle()               # ChWheeledVehicle subsystem
    chassis = bus.GetChassisBody()           # cache: main chassis rigid body, reused every step
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension +
    # steering links created inside the wrapper; tires: TMEASY per wheel.

    # === Terrain (flat rigid patch attached to the wrapper-owned system) ===
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Footprint assertion — wheels must rest on (not through) the terrain ===
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise INIT_Z by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Driver (scripted, attached to this vehicle) ===
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    return bus, veh_obj, chassis, terrain, driver


def main():
    bus, veh_obj, chassis, terrain, driver = build_scene()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("UAZBUS on Rigid Terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.6)   # chase view
        vis.Initialize()                                                 # device first
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(1.0, 1.0, 100, 100,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z + 0.01),
                                       chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)      # review-video frame / motion-log dir

    system = bus.GetSystem()               # cache: fetched once, reused every step

    # === Main loop — render-cadence outer loop, Synchronize/Advance inner batch ===
    # The wrapper's Advance() steps the owned system; we never call
    # system.DoStepDynamics() ourselves (that would double-step).
    data_file = None
    motion_file = None
    times, xs, ys, zs, speeds, throttles = [], [], [], [], [], []
    try:
        data_file = open("simulation_data.csv", "w", newline="")        # main physics log
        motion_file = open("cam/motion_log.csv", "w", newline="")       # per-body motion log
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(
            ["time", "chassis_x", "chassis_y", "chassis_z", "speed", "throttle", "steering"]
        )
        motion_writer.writerow(
            ["time", "body", "x", "y", "z", "vx", "vy", "vz"]
        )

        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                # Subsystem synchronize order: driver, terrain, vehicle, vis.
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                bus.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                # --- log physics this step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_writer.writerow([
                    f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{speed:.5f}", f"{driver_inputs.m_throttle:.4f}",
                    f"{driver_inputs.m_steering:.4f}",
                ])
                motion_writer.writerow([
                    f"{sim_time:.5f}", "chassis",
                    f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
                ])
                times.append(sim_time)
                xs.append(pos.x); ys.append(pos.y); zs.append(pos.z)
                speeds.append(speed); throttles.append(driver_inputs.m_throttle)

                # Advance: driver, terrain, vehicle (steps the system), vis.
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                bus.Advance(TIME_STEP)        # internally calls DoStepDynamics
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= RUN_END:
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
        # Flush partial CSV even if a step diverges — close any open writers here.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing — timeseries plot from the logged arrays ===
    if times:
        t = np.array(times)
        fig, axarr = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axarr[0].plot(t, xs, label="chassis x")
        axarr[0].plot(t, ys, label="chassis y")
        axarr[0].set_ylabel("position (m)")
        axarr[0].legend(loc="best"); axarr[0].grid(True)
        axarr[1].plot(t, zs, color="tab:green", label="chassis z")
        axarr[1].set_ylabel("height (m)")
        axarr[1].legend(loc="best"); axarr[1].grid(True)
        axarr[2].plot(t, speeds, color="tab:red", label="speed")
        axarr[2].plot(t, throttles, color="tab:orange", label="throttle")
        axarr[2].set_ylabel("speed (m/s) / throttle")
        axarr[2].set_xlabel("time (s)")
        axarr[2].legend(loc="best"); axarr[2].grid(True)
        fig.suptitle("UAZBUS on rigid terrain — chassis motion")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        # Report a one-line motion summary for the physics check.
        print(f"steps logged: {len(times)}  "
              f"x: {xs[0]:.2f} -> {xs[-1]:.2f} m  "
              f"max speed: {max(speeds):.2f} m/s  "
              f"z range: [{min(zs):.3f}, {max(zs):.3f}] m")


if __name__ == "__main__":
    main()
