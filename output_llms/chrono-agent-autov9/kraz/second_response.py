"""Kraz tractor + semitrailer performing a double lane change on rigid terrain.

Model
-----
A full Kraz long-haul truck (tractor + articulated semitrailer) built with the
`veh.Kraz` wrapper. The wrapper internally creates and owns an `ChSystemNSC`
(NSC contact), the tractor `ChWheeledVehicle` (chassis, three axles, steering,
driveline, fixed TMEASY-class tires) and the towed semitrailer, all coupled
through the wrapper. A large flat `veh.RigidTerrain` patch provides the road.

Behaviour / objective
----------------------
The truck spawns at world X = -15 m (heading +X) and accelerates from rest, then
executes a time-scripted double lane change (ISO-style): straighten, steer left
into the adjacent lane, hold, steer right back, hold, and recover to centre. The
articulated trailer should track the tractor through the swerve while both stay
upright (no rollover). The chase camera follows a point ahead of the tractor.

Expected: forward translation along +X of tens of metres, two lateral
excursions (left then right) of the chassis Y position, and roll/pitch angles
that stay small (truck remains upright).
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

# === Named constants (geometry / physics / timing) ===
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire sub-step (s)
SIM_END = 16.0                     # total simulated time (s)
RENDER_FPS = 30.0                  # review-video frame rate

INIT_X = -15.0                     # spawn X (world); prompt-specified
INIT_Y = 0.0                       # spawn Y (world)
INIT_Z = 0.5                       # spawn Z; tire radius rests wheels near z=0
TIRE_RADIUS = 0.5588               # Kraz tire radius (m), for the footprint check

TERRAIN_LENGTH = 300.0             # road patch X extent (m)
TERRAIN_WIDTH = 60.0               # road patch Y extent (m)
TERRAIN_FRICTION = 0.9             # road friction coefficient
TERRAIN_RESTITUTION = 0.01         # road restitution

# Chase camera (prompt-specified): track point ahead of tractor, far/high view.
CAM_TRACK_POINT = chrono.ChVector3d(3.0, 0.0, 2.1)
CAM_CHASE_DIST = 25.0
CAM_CHASE_HEIGHT = 10.5

# Double-lane-change schedule (time-based, seconds). Throttle ramps to cruise;
# steering pulses left then right to swap lanes and return.
THROTTLE_RAMP_END = 3.0            # reach cruise throttle by this time
CRUISE_THROTTLE = 0.55             # steady throttle during the manoeuvre
STEER_AMPL = 0.18                  # peak steering magnitude (-1..1)
DLC_T0 = 5.0                       # begin first (left) steer
DLC_T1 = 6.6                       # end left steer / hold in left lane
DLC_T2 = 8.6                       # begin return (right) steer
DLC_T3 = 10.2                      # end right steer / recover to centre

# Derived render cadence — precomputed once (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # steps per frame
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless check
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short validate run


# === Driver (scripted ChDriver subclass) — double lane change vs time ===
class DoubleLaneChangeDriver(veh.ChDriver):
    """Open-loop driver: throttle ramp to cruise + an ISO double-lane-change
    steering profile, all a pure function of simulation time."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Throttle: linear ramp to cruise, then hold.
        if time < THROTTLE_RAMP_END:
            self.SetThrottle(CRUISE_THROTTLE * (time / THROTTLE_RAMP_END))
        else:
            self.SetThrottle(CRUISE_THROTTLE)
        self.SetBraking(0.0)

        # Steering: smooth (sinusoidal) left pulse, hold, then right pulse.
        if DLC_T0 <= time < DLC_T1:
            phase = (time - DLC_T0) / (DLC_T1 - DLC_T0)
            steer = STEER_AMPL * math.sin(math.pi * phase)      # left swerve
        elif DLC_T2 <= time < DLC_T3:
            phase = (time - DLC_T2) / (DLC_T3 - DLC_T2)
            steer = -STEER_AMPL * math.sin(math.pi * phase)     # right swerve
        else:
            steer = 0.0
        self.SetSteering(steer)


def main():
    # === System & bodies (created by the veh.Kraz wrapper) ===
    truck = veh.Kraz()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisCollisionType(veh.CollisionType_NONE)
    truck.SetChassisFixed(False)
    truck.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    truck.SetTireStepSize(TIRE_STEP)         # Kraz tires are a fixed TMEASY-class model
    truck.Initialize()

    # Make the wrapper-created essentials visible as named handles.
    sys = truck.GetSystem()                  # ChSystemNSC owned by the wrapper
    tractor = truck.GetTractor()             # tractor ChWheeledVehicle
    chassis_body = truck.GetTractorChassisBody()  # cache: tractor chassis rigid body
    # bodies: tractor chassis + 3 axles/spindles + trailer; joints: suspension,
    # steering, and the fifth-wheel coupling are created inside the wrapper.

    # Visualization types take TWO args (tractor, trailer) — steering takes one.
    truck.SetChassisVisualizationType(
        chrono.VisualizationType_MESH, chrono.VisualizationType_MESH
    )
    truck.SetSuspensionVisualizationType(
        chrono.VisualizationType_PRIMITIVES, chrono.VisualizationType_PRIMITIVES
    )
    truck.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(
        chrono.VisualizationType_MESH, chrono.VisualizationType_MESH
    )
    truck.SetTireVisualizationType(
        chrono.VisualizationType_MESH, chrono.VisualizationType_MESH
    )

    # Footprint check: wheels must rest on (not far through) the z=0 road.
    wheel_bottom_z = INIT_Z - TIRE_RADIUS
    assert wheel_bottom_z >= -0.1, (
        f"wheels sink into road: bottom z={wheel_bottom_z:.3f}; raise INIT_Z"
    )

    # === Terrain (large flat rigid road) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver (scripted double lane change) ===
    driver = DoubleLaneChangeDriver(tractor)
    driver.Initialize()

    # === Visualization (vehicle-aware Irrlicht: window + sky + camera + lights) ===
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("Kraz Double Lane Change")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(CAM_TRACK_POINT, CAM_CHASE_DIST, CAM_CHASE_HEIGHT)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AttachVehicle(tractor)
        vis.AttachDriver(driver)

    # === Main loop (render-cadence outer loop; Synchronize/Advance per step) ===
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)

    data_f = motion_f = None
    try:
        data_f = open("simulation_data.csv", "w", newline="")
        motion_f = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:       # disk / permission failure on open
        print(f"Could not open CSV output: {exc}")
        raise

    data_w = csv.writer(data_f)
    data_w.writerow(["time", "pos_x", "pos_y", "pos_z", "speed", "roll", "pitch", "steering", "throttle"])
    motion_w = csv.writer(motion_f)
    motion_w.writerow(["time", "body", "x", "y", "z", "roll", "pitch", "yaw"])

    times, xs, ys, speeds, rolls, pitches, steers = [], [], [], [], [], [], []

    frame = 0
    try:
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # --- per-step logging ---
                pos = chassis_body.GetPos()
                rot = chassis_body.GetRot()
                euler = rot.GetCardanAnglesXYZ()      # roll, pitch, yaw (rad)
                speed = tractor.GetSpeed()
                data_w.writerow([
                    f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{speed:.5f}", f"{euler.x:.5f}", f"{euler.y:.5f}",
                    f"{driver_inputs.m_steering:.4f}", f"{driver_inputs.m_throttle:.4f}",
                ])
                motion_w.writerow([
                    f"{time:.5f}", "tractor_chassis",
                    f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{euler.x:.5f}", f"{euler.y:.5f}", f"{euler.z:.5f}",
                ])
                times.append(time); xs.append(pos.x); ys.append(pos.y)
                speeds.append(speed); rolls.append(euler.x); pitches.append(euler.y)
                steers.append(driver_inputs.m_steering)

                # --- advance the full subsystem stack (no DoStepDynamics) ---
                driver.Synchronize(time)
                terrain.Synchronize(time)
                truck.Synchronize(time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                truck.Advance(TIME_STEP)        # internally steps the wrapper system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close CSV writers even if a step diverges mid-run.
        if data_f is not None:
            data_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing (timeseries plot from the logged arrays) ===
    fig, ax = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
    ax[0].plot(times, xs, label="pos_x (forward)")
    ax[0].plot(times, ys, label="pos_y (lateral)")
    ax[0].set_ylabel("position (m)"); ax[0].legend(); ax[0].grid(True)
    ax[1].plot(times, speeds, color="tab:green", label="speed")
    ax[1].set_ylabel("speed (m/s)"); ax[1].legend(); ax[1].grid(True)
    ax[2].plot(times, rolls, label="roll")
    ax[2].plot(times, pitches, label="pitch")
    ax[2].plot(times, steers, label="steering input")
    ax[2].set_ylabel("rad / norm"); ax[2].set_xlabel("time (s)")
    ax[2].legend(); ax[2].grid(True)
    fig.suptitle("Kraz double lane change — trajectory, speed, attitude")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

    print(f"Done: {len(times)} steps, final X={xs[-1]:.2f} m, "
          f"lateral range Y=[{min(ys):.2f},{max(ys):.2f}] m, "
          f"max|roll|={max(abs(r) for r in rolls):.4f} rad")


if __name__ == "__main__":
    main()
