"""FEDA wheeled vehicle performing an ISO double lane change on flat rigid terrain.

Model:
  - System type: NSC (created and owned internally by the veh.FEDA wrapper).
  - Main bodies: FEDA chassis + four wheel spindles/tires (wrapper-created),
    and a single flat RigidTerrain patch acting as the ground.
  - Control: an autonomous veh.ChPathFollowerDriver with cruise control follows a
    veh.DoubleLaneChangePath (ISO standard double lane change). The steering
    controller uses a 5 m look-ahead; the speed controller holds a 10 m/s target.

Expected behavior:
  The vehicle starts near the left edge of a 200 m long terrain patch at
  x = -50 m, accelerates to the 10 m/s target speed, and tracks the double
  lane change path: swerve out of lane, run parallel, swerve back, returning
  to the original lane — all while staying upright on the patch.

Outputs:
  - frames/img_%06d.png   review frames (assembled into cam/review.mp4 by ffmpeg)
  - simulation_data.csv   per-step time / position / speed / steering / throttle
  - cam/motion_log.csv    per-step chassis pose + velocity (motion contract)
  - simulation_timeseries.png  matplotlib summary of the logged channels
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

# === Named constants: geometry / physics / control ===
TIME_STEP = 2e-3                     # integration step (s)
SIM_END = 22.0                       # total simulated time (s)
RENDER_FPS = 50.0                    # review-video frame rate

# Terrain (flat rigid patch) — lengthened so the lane change fits on the patch.
TERRAIN_LENGTH = 200.0               # X size of the rigid patch (m)
TERRAIN_WIDTH = 30.0                 # Y size of the rigid patch (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_YOUNG = 2e7                  # SMC stiffness (Pa)

# Vehicle spawn — moved to the left edge so the maneuver fits within the patch.
VEH_INIT_X = -50.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = 0.5
INIT_LOC = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
INIT_ROT = chrono.QUNIT

# Path-follower / cruise-control parameters.
TARGET_SPEED = 10.0                  # cruise target (m/s)
LOOK_AHEAD = 5.0                     # steering controller look-ahead distance (m)
STEER_GAINS = (0.8, 0.0, 0.0)        # KP, KI, KD for the steering PID
SPEED_GAINS = (0.4, 0.0, 0.05)       # KP, KI, KD for the speed PID

# ISO double lane change path geometry (PyChrono DoubleLaneChangePath).
DLC_LENGTH = 13.5                    # length of each maneuver segment (m)
DLC_WIDTH = 4.0                      # lateral offset between lanes (m)
DLC_OFFSET = 11.0                    # straight run-in before the swerve (m)
DLC_TOTAL = 100.0                    # total path length (m)
DLC_TO_LEFT = True                   # ISO double lane change to the left

# Tire radius (FEDA PAC02 tire) — used for the wheel-bottom support assert.
TIRE_RADIUS = 0.4675

# === Validation gate ===
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

# Derived render cadence — precomputed once (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # physics steps per frame


def build_simulation():
    """Construct the FEDA vehicle wrapper, terrain, driver, path, and visualization."""
    # === System & bodies (created by the veh.FEDA wrapper) ===
    # The wrapper builds and owns the ChSystemSMC, the chassis rigid body, the
    # four wheel spindles, the suspension/steering joints, and the powertrain.
    feda = veh.FEDA()
    feda.SetContactMethod(chrono.ChContactMethod_SMC)
    feda.SetChassisCollisionType(veh.CollisionType_NONE)
    feda.SetChassisFixed(False)
    feda.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    feda.SetTireType(veh.TireModelType_PAC02)        # PAC02 tire on rigid road
    feda.SetTireStepSize(TIME_STEP)
    feda.Initialize()

    feda.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    feda.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    feda.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    feda.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    feda.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = feda.GetSystem()                  # ChSystemSMC owned by the wrapper
    chassis = feda.GetChassisBody()            # cache: main chassis rigid body, reused every step
    vehicle = feda.GetVehicle()                # cache: ChVehicle handle, reused every step
    # spindles: vehicle.GetSpindlePos(axle, side); joints: suspension + steering
    # links created inside the wrapper; powertrain: SIMPLE_MAP engine + transmission.

    # === Terrain (flat rigid patch) ===
    # Single 200 x 30 m patch — long enough to contain the entire lane change.
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(TERRAIN_YOUNG)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Wheel-bottom support assert ===
    # Confirm the four spindles rest on (not through) the flat patch at z=0.
    veh_obj = vehicle
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= -0.10, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z=0.000; raise VEH_INIT_Z"
    )

    # === Driver: path-follower cruise control ===
    # Build the ISO double lane change path starting at the vehicle spawn, then a
    # closed-loop path-follower driver that steers along it at the target speed.
    path_start = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
    path = veh.DoubleLaneChangePath(
        path_start, DLC_LENGTH, DLC_WIDTH, DLC_OFFSET, DLC_TOTAL, DLC_TO_LEFT
    )
    driver = veh.ChPathFollowerDriver(veh_obj, path, "iso_double_lane_change", TARGET_SPEED)
    driver.GetSteeringController().SetLookAheadDistance(LOOK_AHEAD)
    driver.GetSteeringController().SetGains(*STEER_GAINS)
    driver.GetSpeedController().SetGains(*SPEED_GAINS)
    driver.Initialize()

    return feda, vehicle, chassis, system, terrain, driver


def build_visualization(feda, driver):
    """Full vehicle-aware Irrlicht scene: window + chase cam + sky + lights + grid."""
    # === Visualization === full Irrlicht scene (skipped on the headless validation run)
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("FEDA - ISO double lane change")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # chase cam behind chassis
    vis.Initialize()                                              # Initialize FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()                                               # outdoor sky backdrop
    vis.AddTypicalLights()                                        # standard lighting
    vis.AddGrid(1.0, 1.0, 200, 30,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))                   # ground reference grid
    vis.AttachVehicle(feda.GetVehicle())                         # bind chassis/wheel assets
    vis.AttachDriver(driver)                                     # steering/throttle/brake HUD
    return vis


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)      # review video + motion log live here

    feda, vehicle, chassis, system, terrain, driver = build_simulation()

    vis = None
    if not HEADLESS:
        vis = build_visualization(feda, driver)

    # === Main loop ===
    # Render-cadence outer loop: render once per frame, advance physics in an
    # inner batch of RENDER_EVERY steps. The vehicle wrapper's Advance() steps the
    # owned system, so DoStepDynamics is never called directly.
    data_file = None
    motion_file = None
    times, xs, ys, speeds, steers, throttles = [], [], [], [], [], []
    try:
        try:
            data_file = open("simulation_data.csv", "w", newline="")
            motion_file = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
        except (OSError, IOError) as exc:        # disk full / permission denied
            print(f"Could not open CSV output: {exc}")
            raise

        data_writer = csv.writer(data_file)
        data_writer.writerow(["time", "pos_x", "pos_y", "pos_z", "speed",
                              "steering", "throttle", "braking"])
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow(["time", "x", "y", "z", "vx", "vy", "vz", "yaw"])

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

                # --- log physics each step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = vehicle.GetSpeed()
                rot = chassis.GetRot()
                yaw = rot.GetCardanAnglesZYX().z
                data_writer.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}",
                                      f"{pos.y:.5f}", f"{pos.z:.5f}", f"{speed:.5f}",
                                      f"{driver_inputs.m_steering:.5f}",
                                      f"{driver_inputs.m_throttle:.5f}",
                                      f"{driver_inputs.m_braking:.5f}"])
                motion_writer.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}",
                                        f"{pos.y:.5f}", f"{pos.z:.5f}",
                                        f"{vel.x:.5f}", f"{vel.y:.5f}",
                                        f"{vel.z:.5f}", f"{yaw:.5f}"])
                times.append(sim_time)
                xs.append(pos.x); ys.append(pos.y); speeds.append(speed)
                steers.append(driver_inputs.m_steering)
                throttles.append(driver_inputs.m_throttle)

                # --- advance the full subsystem stack (no DoStepDynamics) ---
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                vehicle.Synchronize(sim_time, driver_inputs, terrain)
                if vis is not None:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                feda.Advance(TIME_STEP)        # advances the wrapper-owned system
                if vis is not None:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
        import traceback
        traceback.print_exc()
        print(f"Simulation aborted: {exc}")
        raise
    finally:
        # Flush partial CSV even if a step diverges.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing ===
    # Plot the logged channels vs time to a single summary PNG.
    if times:
        t = np.array(times)
        fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        ax[0].plot(t, xs, label="pos_x")
        ax[0].plot(t, ys, label="pos_y (lateral)")
        ax[0].set_ylabel("position (m)")
        ax[0].legend(); ax[0].grid(True)
        ax[1].plot(t, speeds, color="tab:green")
        ax[1].axhline(TARGET_SPEED, color="k", ls="--", lw=0.8, label="target")
        ax[1].set_ylabel("speed (m/s)")
        ax[1].legend(); ax[1].grid(True)
        ax[2].plot(t, steers, label="steering")
        ax[2].plot(t, throttles, label="throttle")
        ax[2].set_ylabel("driver input"); ax[2].set_xlabel("time (s)")
        ax[2].legend(); ax[2].grid(True)
        fig.suptitle("FEDA ISO double lane change")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=90)
        plt.close(fig)

    print(f"Done. Logged {len(times)} steps; final x={xs[-1]:.2f} m, "
          f"lateral y={ys[-1]:.2f} m, speed={speeds[-1]:.2f} m/s")


if __name__ == "__main__":
    main()
