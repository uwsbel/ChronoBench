"""HMMWV circular-path-following simulation (PyChrono 9.0.x, Irrlicht).

Models a full HMMWV wheeled vehicle (veh.HMMWV_Full wrapper, contact method SMC)
driving autonomously around a circular Bezier path on a flat RigidTerrain patch.
The vehicle is driven by a scripted veh.ChDriver subclass that applies a CONSTANT
throttle of 0.3 and computes its steering with a PID ChPathSteeringController that
tracks the circular path. The terrain patch is 200 m square so the circular path
fits comfortably inside the terrain footprint.

Bodies/subsystems (created by the wrapper): chassis rigid body, four wheel
spindles + tires, suspension/steering links, plus a RigidTerrain ground patch.

Visualization helpers requested:
  * two fixed marker balls placed on the circular path so the path is visible;
  * a green sentinel sphere + a red target sphere that follow the steering
    controller's sentinel/target locations every frame.

Expected behavior: the chassis accelerates from rest under constant throttle and
the PID steering keeps it tracking the circle — it should travel a curved path,
remain upright, and accumulate substantial displacement from the spawn point.
"""

import os
import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / control parameters (no bare literals downstream)
TIME_STEP = 2e-3                     # integration step (s)
SIM_END = 25.0                       # simulated duration (s)
RENDER_FPS = 50.0                    # review-video frame rate

TERRAIN_LENGTH = 200.0               # X extent of the rigid terrain patch (m)
TERRAIN_WIDTH = 200.0                # Y extent of the rigid terrain patch (m)
TERRAIN_FRICTION = 0.9               # tire/ground friction coefficient
TERRAIN_RESTITUTION = 0.01           # ground restitution
TERRAIN_YOUNG = 2e7                  # SMC contact stiffness (Pa)

PATH_RADIUS = 30.0                   # circular path radius (m) — fits in 200 m terrain
PATH_RUN_IN = 10.0                   # straight run-in before the circle starts (m)
PATH_TURNS = 5                       # number of full laps the path describes
PATH_LEFT_TURN = True                # turn direction

CONST_THROTTLE = 0.3                 # constant throttle requested by the maneuver
STEER_KP = 0.8                       # PID steering proportional gain
STEER_KI = 0.0                       # PID steering integral gain
STEER_KD = 0.0                       # PID steering derivative gain
STEER_LOOKAHEAD = 5.0                # steering look-ahead distance (m)

INIT_X = 0.0                         # chassis spawn X (m)
INIT_Y = 0.0                         # chassis spawn Y (m)
SUSPENSION_REF_HEIGHT = 0.5          # chassis-origin height above wheel-bottom at rest (m)
TERRAIN_TOP_Z = 0.0                  # flat terrain top plane (m)
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT
TIRE_RADIUS = 0.46                   # nominal HMMWV tire radius for footprint assert (m)
ZTOL = 0.1                           # allowed wheel-bottom clearance vs terrain (m)

MARKER_RADIUS = 0.4                  # radius of the two path-marker balls (m)
SENTINEL_RADIUS = 0.25               # green sentinel sphere radius (m)
TARGET_RADIUS = 0.25                 # red target sphere radius (m)

# Derived render cadence — precomputed once (never recomputed in the loop).
RENDER_STEPS = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# Headless validation gate: fast, windowless physics check.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 1.0) if HEADLESS else SIM_END          # short physics check when validating


# === Scripted driver === constant-throttle + PID path-steering ChDriver subclass
class CircleThrottleSteerDriver(veh.ChDriver):
    """Applies constant throttle and PID path-tracking steering each step.

    Steering comes from a ChPathSteeringController (PID) tracking the circular
    path; throttle is held constant; brake stays released.
    """

    def __init__(self, vehicle, path):
        super().__init__(vehicle)
        self._veh = vehicle                              # cache: vehicle ref reused every step
        self._steer_ctrl = veh.ChPathSteeringController(path)
        self._steer_ctrl.SetLookAheadDistance(STEER_LOOKAHEAD)
        self._steer_ctrl.SetGains(STEER_KP, STEER_KI, STEER_KD)
        self._initialized = False

    def Synchronize(self, time):
        ref_frame = self._veh.GetRefFrame()              # chassis moving frame for the PID
        if not self._initialized:
            self._steer_ctrl.Reset(ref_frame)
            self._initialized = True
        steering = self._steer_ctrl.Advance(ref_frame, time, TIME_STEP)
        steering = max(-1.0, min(1.0, steering))         # clamp to valid range
        self.SetSteering(steering)
        self.SetThrottle(CONST_THROTTLE)                 # constant throttle 0.3
        self.SetBraking(0.0)

    def GetSteeringController(self):
        return self._steer_ctrl


def main():
    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    # The wrapper creates and owns the ChSystemSMC plus the chassis body, the four
    # wheel spindles + tires, and the suspension/steering joint links internally.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)          # grip model on the rigid road
    hmmwv.SetTireStepSize(TIME_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = hmmwv.GetSystem()                           # cache: ChSystemSMC owned by the wrapper
    chassis = hmmwv.GetChassisBody()                     # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()                         # cache: ChVehicle handle, reused for spindles/ref frame
    # spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links inside the wrapper.

    # Footprint sanity check: wheel bottoms must rest on (not through) the terrain.
    lowest_spindle_z = min(
        veh_obj.GetSpindlePos(axle, side).z
        for axle in range(veh_obj.GetNumberAxles())
        for side in (veh.LEFT, veh.RIGHT)
    )
    wheel_bottom_z = lowest_spindle_z - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Terrain === flat 200 m square RigidTerrain patch so the circle fits inside
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(TERRAIN_YOUNG)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Path === circular Bezier path the driver follows
    path_start = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
    path = veh.CirclePath(path_start, PATH_RADIUS, PATH_RUN_IN, PATH_LEFT_TURN, PATH_TURNS)

    # === Driver === constant throttle + PID steering toward the circular path
    driver = CircleThrottleSteerDriver(veh_obj, path)
    driver.Initialize()
    steer_ctrl = driver.GetSteeringController()          # cache: controller fetched once, reused every frame

    # === Path-visualization markers === two fixed balls on the circle + sentinel/target spheres
    # Two marker balls placed at diametrically opposite points of the circle so the
    # path is visible. The circle center sits PATH_RADIUS to the left of the run-in.
    circle_center_y = INIT_Y + (PATH_RADIUS if PATH_LEFT_TURN else -PATH_RADIUS)
    circle_center_x = INIT_X + PATH_RUN_IN

    def make_marker(pos, radius, color):
        body = chrono.ChBody()
        body.SetFixed(True)
        body.SetPos(pos)
        shp = chrono.ChVisualShapeSphere(radius)
        shp.SetColor(color)
        body.AddVisualShape(shp, chrono.ChFramed())
        system.AddBody(body)
        return body

    make_marker(
        chrono.ChVector3d(circle_center_x, circle_center_y + PATH_RADIUS, INIT_Z),
        MARKER_RADIUS, chrono.ChColor(0.1, 0.2, 1.0),
    )
    make_marker(
        chrono.ChVector3d(circle_center_x, circle_center_y - PATH_RADIUS, INIT_Z),
        MARKER_RADIUS, chrono.ChColor(0.1, 0.2, 1.0),
    )
    sentinel_marker = make_marker(path_start, SENTINEL_RADIUS, chrono.ChColor(0.0, 1.0, 0.0))
    target_marker = make_marker(path_start, TARGET_RADIUS, chrono.ChColor(1.0, 0.0, 0.0))

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV Circular Path Follower")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.8)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(
            1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4),
        )                                                # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    os.makedirs("frames", exist_ok=True)                 # guard against missing output dir
    os.makedirs("cam", exist_ok=True)                    # review video output dir

    data_file = None
    motion_file = None
    times, speeds, xs, ys = [], [], [], []
    try:
        data_file = open("simulation_data.csv", "w", newline="")          # closed in finally
        motion_file = open("cam/motion_log.csv", "w", newline="")         # closed in finally
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(["time", "x", "y", "z", "speed", "steering", "throttle"])
        motion_writer.writerow(["time", "x", "y", "z", "vx", "vy", "vz", "speed"])

        # === Main loop === render-cadence outer loop; Synchronize/Advance the subsystem stack
        step = 0
        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            time = system.GetChTime()

            if not HEADLESS and step % RENDER_STEPS == 0:
                # Move the controller markers to the live sentinel/target points.
                sentinel_marker.SetPos(steer_ctrl.GetSentinelLocation())
                target_marker.SetPos(steer_ctrl.GetTargetLocation())
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            driver_inputs = driver.GetInputs()

            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(time, driver_inputs)

            # Log physics each step.
            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            speed = veh_obj.GetSpeed()
            data_writer.writerow([
                f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                f"{speed:.5f}", f"{driver_inputs.m_steering:.5f}",
                f"{driver_inputs.m_throttle:.5f}",
            ])
            motion_writer.writerow([
                f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}", f"{speed:.5f}",
            ])
            times.append(time); speeds.append(speed); xs.append(pos.x); ys.append(pos.y)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)                      # advances the wrapper-owned system
            if not HEADLESS:
                vis.Advance(TIME_STEP)
            step += 1

    except (OSError, IOError) as exc:                    # disk / permission failure on CSV I/O
        import traceback; traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:            # solver divergence / bad state
        import traceback; traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === plot trajectory + speed vs time from the logged data
    if times:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        ax1.plot(xs, ys, "b-")
        ax1.set_title("Chassis trajectory (XY)")
        ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
        ax1.axis("equal"); ax1.grid(True)
        ax2.plot(times, speeds, "r-")
        ax2.set_title("Chassis speed vs time")
        ax2.set_xlabel("time (s)"); ax2.set_ylabel("speed (m/s)")
        ax2.grid(True)
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    print(f"Done: steps simulated to t={times[-1] if times else 0.0:.3f}s, "
          f"final speed={speeds[-1] if speeds else 0.0:.3f} m/s, "
          f"displacement=({xs[-1] - INIT_X:.2f}, {ys[-1] - INIT_Y:.2f}) m")


if __name__ == "__main__":
    main()
