"""Sedan cruising along a highway-mesh road under closed-loop speed control.

Model: a single catalog ``veh.Sedan`` (wrapper-managed, owns its own
``ChSystemNSC``) driving on a rigid terrain whose surface is a triangle-mesh
highway road. Tires are TMEASY (slip/grip force model) so the car actually
translates on the rigid road.

Control:
  * Throttle is closed-loop. A PID controller drives the chassis forward speed
    toward a constant REFERENCE_SPEED using the instantaneous speed error; its
    output is clamped to a valid throttle in [0, 1].
  * Steering is an open-loop ramp that grows linearly from 0 to a held target
    over STEER_RESPONSE_TIME seconds, so the car gently curves once it is
    rolling.

Expected behavior: the sedan starts at rest at the south end of the highway
facing +Y, the PID throttle accelerates it up toward the reference speed and
holds it there, and the steering ramp curves the path after the first seconds.
Logged: time, forward speed, reference speed, speed error, throttle, steering,
chassis position. The run writes simulation_data.csv, cam/motion_log.csv and a
matplotlib summary PNG.
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

# === Named constants: timing, geometry, control gains ===
# Finer step sizes for finer control fidelity.
TIME_STEP = 2.0e-4              # physics step (s) — small for fine control
RENDER_STEP_SIZE = 1.0 / 60.0  # one review frame every 1/60 s
SIM_END = 12.0                 # total simulated time (s)

# Initial vehicle location and orientation (highway runs along +Y; spawn south,
# face +Y by rotating 90 deg about Z).
INIT_X = 0.0
INIT_Y = -60.0
SUSPENSION_REF_HEIGHT = 0.20   # chassis-origin height giving wheel bottoms ~ road top
INIT_HEADING = math.pi / 2.0   # face +Y down the highway

# Speed-tracking PID throttle controller.
REFERENCE_SPEED = 10.0         # target forward speed (m/s)
KP_THROTTLE = 0.40             # proportional gain on speed error
KI_THROTTLE = 0.08             # integral gain (removes steady-state error)
KD_THROTTLE = 0.02             # derivative gain (damps overshoot)

# Steering ramp: reach the held steering target over this response time.
STEER_RESPONSE_TIME = 5.0      # seconds from 0 to STEER_TARGET
STEER_TARGET = 0.35            # held steering command (-1..1)

# Terrain / tire geometry.
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TIRE_RADIUS = 0.3266           # Sedan front tire radius (from tire model)
ZTOL = 0.08                    # allowed wheel-bottom clearance/overlap vs road

HIGHWAY_VIS_MESH = "synchrono/meshes/Highway_vis.obj"
HIGHWAY_COL_MESH = "synchrono/meshes/Highway_col.obj"

# Derived once (precomputed — never recompute inside the loop).
render_steps = max(1, math.ceil(RENDER_STEP_SIZE / TIME_STEP))  # precomputed once
init_z = SUSPENSION_REF_HEIGHT
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, init_z)
init_rot = chrono.QuatFromAngleZ(INIT_HEADING)

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast windowless validation run


# === Driver: PID-throttle + ramped-steering scripted controller ===
# A ChDriver subclass whose Synchronize() computes throttle from a PID law on
# the forward-speed error and steering from a linear ramp. State (integral,
# previous error/time) lives on the instance.
class SpeedControlDriver(veh.ChDriver):
    def __init__(self, vehicle, get_speed):
        super().__init__(vehicle)
        self._get_speed = get_speed       # cache: speed getter bound once
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = 0.0

    def Synchronize(self, time):
        # --- PID throttle on speed error ---
        speed = self._get_speed()
        error = REFERENCE_SPEED - speed
        dt = time - self._prev_time
        if dt > 0.0:
            self._integral += error * dt
            derivative = (error - self._prev_error) / dt
        else:
            derivative = 0.0
        out = (KP_THROTTLE * error
               + KI_THROTTLE * self._integral
               + KD_THROTTLE * derivative)
        throttle = min(1.0, max(0.0, out))   # clamp to valid throttle range
        # Anti-windup: do not accumulate integral once throttle is saturated.
        if out > 1.0 or out < 0.0:
            self._integral -= error * dt if dt > 0.0 else 0.0
        self.SetThrottle(throttle)
        self.SetBraking(0.0)

        # --- Steering ramp: 0 -> STEER_TARGET over STEER_RESPONSE_TIME ---
        ramp = min(1.0, time / STEER_RESPONSE_TIME)
        self.SetSteering(STEER_TARGET * ramp)

        self._prev_error = error
        self._prev_time = time


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing frame output dir
    os.makedirs("cam", exist_ok=True)       # review video / motion log output dir

    # === Vehicle wrapper (creates + owns its ChSystemNSC) ===
    sedan = veh.Sedan()
    sedan.SetContactMethod(chrono.ChContactMethod_NSC)
    sedan.SetChassisCollisionType(veh.CollisionType_NONE)
    sedan.SetChassisFixed(False)
    sedan.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    sedan.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire so the car drives
    sedan.SetTireStepSize(TIME_STEP)
    sedan.Initialize()

    sedan.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    sedan.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    sedan.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    sedan.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    sedan.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.Sedan wrapper) ===
    sys = sedan.GetSystem()                 # ChSystemNSC owned by the wrapper
    veh_obj = sedan.GetVehicle()            # cache: vehicle handle reused below
    chassis = sedan.GetChassisBody()        # cache: main chassis rigid body, reused every step
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); suspension + steering
    # joints are created inside the wrapper; terrain patch body added below.

    # Footprint assert: wheel bottoms must rest on (not through) the road top.
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= -ZTOL, (
        f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs road top z=0.0; raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
    )

    # === Terrain: rigid terrain with a highway triangle-mesh road surface ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,                                  # mesh at origin, no rotation
        chrono.GetChronoDataFile(HIGHWAY_COL_MESH),       # collision mesh (road surface)
        True,                                             # connected mesh
        0.0,                                              # sweep sphere radius
        True,                                             # visualization on
    )
    # Overlay the higher-detail visual highway mesh for the review video.
    patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/concrete.jpg"), 10, 60)
    terrain.Initialize()

    # === Driver: closed-loop PID throttle + ramped steering ===
    speed_getter = veh_obj.GetSpeed       # cache: bound speed getter for the PID
    driver = SpeedControlDriver(veh_obj, speed_getter)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("Sedan highway speed-control cruise")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.6)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(INIT_X - 8.0, INIT_Y - 8.0, 4.0),
                      chrono.ChVector3d(INIT_X, INIT_Y, 0.5))
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 30, 30,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.35, 0.35, 0.35))   # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

    # === Main loop === render-cadence outer loop; full subsystem Sync/Advance inner batch
    data_file = None
    motion_file = None
    times, speeds, refs, throttles, steers = [], [], [], [], []
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(
            ["time", "speed", "ref_speed", "speed_error", "throttle", "steering"])
        motion_writer.writerow(
            ["time", "body", "x", "y", "z", "speed", "vx", "vy", "vz"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(render_steps):
                time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # Log physics every step.
                speed = veh_obj.GetSpeed()
                error = REFERENCE_SPEED - speed
                data_writer.writerow([
                    f"{time:.5f}", f"{speed:.5f}", f"{REFERENCE_SPEED:.5f}",
                    f"{error:.5f}", f"{driver_inputs.m_throttle:.5f}",
                    f"{driver_inputs.m_steering:.5f}"])
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                motion_writer.writerow([
                    f"{time:.5f}", "chassis", f"{pos.x:.5f}", f"{pos.y:.5f}",
                    f"{pos.z:.5f}", f"{speed:.5f}", f"{vel.x:.5f}",
                    f"{vel.y:.5f}", f"{vel.z:.5f}"])
                times.append(time)
                speeds.append(speed)
                refs.append(REFERENCE_SPEED)
                throttles.append(driver_inputs.m_throttle)
                steers.append(driver_inputs.m_steering)

                # Advance the full subsystem stack (no DoStepDynamics here —
                # sedan.Advance steps the wrapper-owned system).
                driver.Synchronize(time)
                terrain.Synchronize(time)
                sedan.Synchronize(time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                sedan.Advance(TIME_STEP)        # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= run_end:
                    break
    except (OSError, IOError) as exc:        # disk / permission failure on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close writers even if a step diverges mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === plot the logged time series
    if times:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax1.plot(times, speeds, label="speed (m/s)")
        ax1.plot(times, refs, "--", label="reference speed (m/s)")
        ax1.set_ylabel("speed (m/s)")
        ax1.legend(loc="best")
        ax1.grid(True)
        ax2.plot(times, throttles, label="throttle")
        ax2.plot(times, steers, label="steering")
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("command")
        ax2.legend(loc="best")
        ax2.grid(True)
        fig.suptitle("Sedan highway PID speed control + steering ramp")
        fig.tight_layout()
        with open("simulation_timeseries.png", "wb") as png:
            fig.savefig(png, dpi=110)
        plt.close(fig)

    final_speed = speeds[-1] if speeds else 0.0
    print(f"done: steps_logged={len(times)} final_speed={final_speed:.3f} m/s "
          f"final_pos_y={chassis.GetPos().y:.3f}")


if __name__ == "__main__":
    main()
