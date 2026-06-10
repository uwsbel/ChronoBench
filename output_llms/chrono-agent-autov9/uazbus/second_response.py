"""UAZBUS double-lane-change maneuver on a flat rigid concrete road (PyChrono 9.0.1).

Model
-----
- A UAZBUS wheeled vehicle (catalog wrapper `veh.UAZBUS`), which internally owns
  an NSC ChSystem, the chassis rigid body, four spindle/wheel bodies, the
  suspension + steering joints, the engine/transmission/driveline, and the tires.
- A large flat RigidTerrain patch (Bullet rigid contacts) textured with
  concrete.jpg, sized so the bus stays on the patch throughout the maneuver.
- TMEASY deformable-contact tire model on the rigid road (good slip/grip curve).

Control / expected behavior
---------------------------
The bus spawns at world X = -40 m, heading +X, and accelerates from rest. A
scripted time-based driver performs a double lane change: it builds speed on a
straight line, executes two opposite-phase steering pulses (steer left, then
right, then back) to weave the vehicle across to an adjacent lane and back, and
finally releases throttle and brakes to a stop. The chassis should translate a
large net distance along +X while its lateral (Y) position traces the
characteristic out-and-back double-lane-change profile, all four wheels staying
on the concrete patch, and the bus remaining upright (roll/pitch small).

System type: NSC (UAZBUS catalog default). Renderer: Irrlicht chase camera.
Outputs: frames/img_*.png review frames, simulation_data.csv, cam/motion_log.csv,
and simulation_timeseries.png.
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

# === Constants (geometry / physics / control) ===
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire sub-step (s); TMEASY needs its own step
SIM_END = 18.0                     # total simulated time (s)
RENDER_FPS = 30.0                  # review-video frame rate

# Spawn: prompt-specified initial vehicle position (heading +X).
INIT_X = -40.0
INIT_Y = 0.0
INIT_Z = 0.5                       # chassis-origin height so wheels rest on z=0 road
INIT_LOC = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.QUNIT

# Vehicle geometry (introspected from this catalog model after Initialize).
TIRE_RADIUS = 0.372                # UAZBUS tire radius (m)
ROAD_TOP_Z = 0.0                   # flat patch top plane (z = 0)
ZTOL = 0.05                        # allowed wheel-bottom clearance/overlap vs road

# Terrain patch: large enough that the whole maneuver stays on-patch. The bus
# travels along +X from -40; size the patch generously in X and wide in Y.
TERRAIN_LENGTH = 250.0             # X extent (m)
TERRAIN_WIDTH = 40.0               # Y extent (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# Double-lane-change control schedule (scripted, time-based).
ACCEL_END = 4.0                    # build speed straight until this time (s)
STEER_AMP = 0.42                   # peak steering magnitude (-1..1)
LANE1_START = 4.0                  # begin first (left) lane change
LANE1_END = 6.0
HOLD_END = 7.0                     # straight hold in the adjacent lane
LANE2_START = 7.0                  # begin return (right) lane change
LANE2_END = 9.0
CRUISE_THROTTLE = 0.55             # throttle while maneuvering
BRAKE_START = 12.0                 # release throttle and brake to a stop
ROLLOVER_LIMIT = 0.6               # |roll|/|pitch| (rad) sanity bound (~34 deg)

# Derived once (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless check
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check


# === Scripted double-lane-change driver (subclass of veh.ChDriver) ===
# A custom time-based control law: accelerate straight, two opposite-phase
# steering pulses (left then right) for the lane changes, then brake to a stop.
def _smooth_pulse(t, t0, t1):
    """One full sine period in [t0, t1] (0 -> +1 -> 0 -> -1 -> 0); 0 outside."""
    if t < t0 or t > t1:
        return 0.0
    return math.sin(2.0 * math.pi * (t - t0) / (t1 - t0))


class DoubleLaneChangeDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Throttle / braking phases.
        if time < BRAKE_START:
            self.SetThrottle(CRUISE_THROTTLE)
            self.SetBraking(0.0)
        else:
            self.SetThrottle(0.0)
            self.SetBraking(0.7)                       # decelerate to a stop

        # Steering: two opposite-phase pulses produce the lane-change weave.
        steer = 0.0
        if LANE1_START <= time <= LANE1_END:
            steer = STEER_AMP * _smooth_pulse(time, LANE1_START, LANE1_END)
        elif LANE2_START <= time <= LANE2_END:
            steer = -STEER_AMP * _smooth_pulse(time, LANE2_START, LANE2_END)
        self.SetSteering(steer)


def main():
    # === System & bodies (created by the veh.UAZBUS wrapper) ===
    # The wrapper builds and OWNS the ChSystemNSC plus the chassis, four
    # spindle/wheel bodies, suspension + steering joints, and the powertrain.
    bus = veh.UAZBUS()
    bus.SetContactMethod(chrono.ChContactMethod_NSC)
    bus.SetChassisCollisionType(veh.CollisionType_NONE)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    bus.SetTireType(veh.TireModelType_TMEASY)          # slip/grip curve for steering
    bus.SetTireStepSize(TIRE_STEP)
    bus.Initialize()

    bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = bus.GetSystem()                # ChSystemNSC owned by the wrapper
    veh_obj = bus.GetVehicle()              # cache: vehicle handle, reused below
    chassis = bus.GetChassisBody()          # cache: main chassis rigid body
    # spindles/wheels: veh_obj.GetSpindlePos(axle, side); joints: inside wrapper.

    # === Footprint assert (wheels rest on the road, not through it) ===
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= ROAD_TOP_Z - ZTOL, (
        f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs road top z={ROAD_TOP_Z:.3f}; raise INIT_Z by "
        f"{ROAD_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Terrain (large flat rigid concrete patch) ===
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/concrete.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.Initialize()

    # === Driver (scripted double-lane-change maneuver) ===
    driver = DoubleLaneChangeDriver(veh_obj)
    driver.Initialize()

    # === Visualization (Irrlicht chase camera; full standard scene) ===
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("UAZBUS Double Lane Change")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
        vis.Initialize()                                # Irrlicht: Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddCamera(chrono.ChVector3d(INIT_X - 8, -8, 4), INIT_LOC)
        vis.AddGrid(2.0, 2.0, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.45, 0.45, 0.45))   # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Output setup ===
    os.makedirs("frames", exist_ok=True)        # review frames -> ffmpeg mp4
    os.makedirs("cam", exist_ok=True)           # motion log lives under cam/

    sim_csv = None
    motion_csv = None
    times, xs, ys, speeds, rolls, pitches = [], [], [], [], [], []

    try:
        try:
            sim_csv = open("simulation_data.csv", "w", newline="")
            motion_csv = open("cam/motion_log.csv", "w", newline="")
        except (OSError, IOError) as exc:           # disk full / permission
            print(f"failed to open CSV output: {exc}")
            raise

        sim_writer = csv.writer(sim_csv)
        sim_writer.writerow(["time", "x", "y", "z", "speed",
                             "roll", "pitch", "yaw",
                             "steering", "throttle", "braking"])
        motion_writer = csv.writer(motion_csv)
        motion_writer.writerow(["time", "body", "x", "y", "z",
                                "vx", "vy", "vz", "roll", "pitch", "yaw"])

        # === Main loop (render-cadence outer loop; Synchronize/Advance inner) ===
        step = 0
        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive idx
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                # Log physics state every step (cached chassis handle).
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                rot = chassis.GetRot()
                rpy = rot.GetCardanAnglesXYZ()       # roll, pitch, yaw (rad)
                speed = veh_obj.GetSpeed()
                sim_writer.writerow([
                    f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{speed:.5f}", f"{rpy.x:.5f}", f"{rpy.y:.5f}", f"{rpy.z:.5f}",
                    f"{driver_inputs.m_steering:.4f}",
                    f"{driver_inputs.m_throttle:.4f}",
                    f"{driver_inputs.m_braking:.4f}",
                ])
                motion_writer.writerow([
                    f"{sim_time:.5f}", "chassis",
                    f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
                    f"{rpy.x:.5f}", f"{rpy.y:.5f}", f"{rpy.z:.5f}",
                ])
                times.append(sim_time)
                xs.append(pos.x); ys.append(pos.y); speeds.append(speed)
                rolls.append(rpy.x); pitches.append(rpy.y)

                # Upright sanity check (NaN-safe): the bus must not roll over.
                assert abs(rpy.x) < ROLLOVER_LIMIT and abs(rpy.y) < ROLLOVER_LIMIT, (
                    f"vehicle rolled/pitched over at t={sim_time:.2f}: "
                    f"roll={rpy.x:.2f} pitch={rpy.y:.2f} rad"
                )

                # Synchronize then Advance the full subsystem stack.
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                bus.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                bus.Advance(TIME_STEP)            # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                step += 1
                if system.GetChTime() >= RUN_END:
                    break

    except (RuntimeError, ValueError) as exc:       # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged.
        if sim_csv is not None:
            sim_csv.close()
        if motion_csv is not None:
            motion_csv.close()

    # === Post-processing (timeseries plot from the logged arrays) ===
    if times:
        t = np.asarray(times)
        fig, axarr = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axarr[0].plot(t, xs, label="x (m)")
        axarr[0].plot(t, ys, label="y (m)")
        axarr[0].set_ylabel("position (m)")
        axarr[0].legend(); axarr[0].grid(True)
        axarr[0].set_title("UAZBUS double lane change")
        axarr[1].plot(t, speeds, color="tab:green")
        axarr[1].set_ylabel("speed (m/s)"); axarr[1].grid(True)
        axarr[2].plot(t, np.degrees(rolls), label="roll (deg)")
        axarr[2].plot(t, np.degrees(pitches), label="pitch (deg)")
        axarr[2].set_ylabel("attitude (deg)"); axarr[2].set_xlabel("time (s)")
        axarr[2].legend(); axarr[2].grid(True)
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        net_dx = xs[-1] - xs[0]
        lateral_span = max(ys) - min(ys)
        print(f"steps={len(times)} net_dx={net_dx:.2f} m "
              f"lateral_span={lateral_span:.2f} m final_speed={speeds[-1]:.2f} m/s")


if __name__ == "__main__":
    main()
