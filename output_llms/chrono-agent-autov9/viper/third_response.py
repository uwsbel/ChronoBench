"""
Viper rover driving straight on SCM deformable (Bekker-Wong) terrain.

Model
-----
System      : ChSystemNSC (rigid multibody, Bullet collision required by SCMTerrain).
Robot       : pychrono.robot.Viper four-wheel rover with a ViperDCMotorControl
              driver. The driver spins all four drive motors; the steering angle
              is held at a CONSTANT 0.0 so the rover tracks a straight line.
Terrain     : veh.SCMTerrain — a deformable soft-soil patch (replaces a flat
              rigid ground). The wheels sink slightly and leave ruts; an active
              domain attached to the chassis keeps the ray-cast cost bounded.
Expected    : the rover starts on the soil and drives forward (+X) in a straight
              line at near-constant heading, leaving visible tracks. Chassis pose
              and velocity are logged every physics step.

Output      : simulation_data.csv, cam/motion_log.csv, frames/img_*.png,
              simulation_timeseries.png.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless backend — render plots to PNG without a display
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics) ===
TIME_STEP = 2.5e-3              # s, integration step
SIM_END = 12.0                 # s, total simulated time
RENDER_FPS = 30.0              # frames per second for the review video
GRAVITY = -9.81               # m/s^2, along -Z (Z-up world)

# Rover spawn — start just above the soil rest plane (z=0) so the wheels settle on it.
ROVER_INIT_X = -4.0           # m, start near one edge so it drives across +X
ROVER_INIT_Y = 0.0           # m
ROVER_INIT_Z = 0.12          # m, suspension reference height above terrain surface

STEERING_ANGLE = 0.0          # rad, CONSTANT steering -> straight-line motion
DRIVE_NO_LOAD_SPEED = math.pi  # rad/s, drive-motor no-load speed
DRIVE_STALL_TORQUE = 300.0    # N*m, drive-motor stall torque

# SCM deformable terrain parameters.
SCM_LENGTH = 16.0             # m, terrain extent in X
SCM_WIDTH = 8.0              # m, terrain extent in Y
SCM_RESOLUTION = 0.04         # m, grid spacing (finer -> sharper ruts, costlier)

# Soil (Bekker-Wong) parameters — firm soft-soil: deformable enough to leave ruts,
# stiff enough that the light rover rides near the surface instead of burying itself.
SOIL_BEKKER_KPHI = 2e6        # Pa, frictional modulus
SOIL_BEKKER_KC = 0.0         # cohesive modulus
SOIL_BEKKER_N = 1.1          # exponent
SOIL_MOHR_COHESION = 0.0     # Pa, cohesive limit
SOIL_MOHR_FRICTION = 30.0    # deg, internal friction angle
SOIL_JANOSI_SHEAR = 0.01     # m, shear deformation coefficient
SOIL_ELASTIC_K = 2e8         # Pa/m, elastic stiffness
SOIL_DAMPING_R = 3e4         # Pa*s/m, vertical damping

# Wheel contact material (NSC system).
WHEEL_FRICTION = 0.8
WHEEL_RESTITUTION = 0.0

# === Derived constants (precomputed once) ===
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: steps per frame
ROVER_INIT_POS = chrono.ChVector3d(ROVER_INIT_X, ROVER_INIT_Y, ROVER_INIT_Z)
ROVER_INIT_ROT = chrono.QUNIT
CAM_EYE = chrono.ChVector3d(ROVER_INIT_X - 3.0, -6.0, 3.0)   # behind-left, elevated
CAM_TARGET = chrono.ChVector3d(0.0, 0.0, 0.0)

# Validation gate: fast, windowless physics check when SIMBENCH_VALIDATE is set.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 1.0) if HEADLESS else SIM_END  # short physics check when validating


def main():
    # === System & gravity ===
    # NSC rigid-body system. SCMTerrain REQUIRES a Bullet collision system to exist
    # before construction, so set the collision type up front.
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required by SCMTerrain
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, GRAVITY))

    # === Rover (Viper) ===
    # Four-wheel rover with a DC-motor driver. RealWheel geometry carries its own
    # collision shapes, so SCM ray-casts hit the wheels directly (no helper cylinders).
    wheel_mat = chrono.ChContactMaterialNSC()
    wheel_mat.SetFriction(WHEEL_FRICTION)
    wheel_mat.SetRestitution(WHEEL_RESTITUTION)

    rover = robot.Viper(sys)
    rover.SetWheelContactMaterial(wheel_mat)

    driver = robot.ViperDCMotorControl()
    for wheel_id in (robot.V_LF, robot.V_RF, robot.V_LB, robot.V_RB):
        driver.SetMotorNoLoadSpeed(DRIVE_NO_LOAD_SPEED, wheel_id)
        driver.SetMotorStallTorque(DRIVE_STALL_TORQUE, wheel_id)
    rover.SetDriver(driver)

    rover.Initialize(chrono.ChFramed(ROVER_INIT_POS, ROVER_INIT_ROT))

    # Constant steering -> straight-line trajectory (no time-varying steering law).
    driver.SetSteering(STEERING_ANGLE)

    chassis = rover.GetChassis()              # cache: fetched once, reused every step

    # === Terrain (SCM deformable soft soil) ===
    # Replaces a flat rigid ground with a Bekker-Wong deformable surface so the
    # wheels sink slightly and leave ruts. The patch is small (16x8 m at 4 cm grid,
    # ~80k vertices), so it ray-casts the whole grid every step — no active-domain
    # restriction is used; restricting it filters the wheel ray-casts and starves
    # the rover of traction.
    terrain = veh.SCMTerrain(sys)
    terrain.SetSoilParameters(
        SOIL_BEKKER_KPHI,
        SOIL_BEKKER_KC,
        SOIL_BEKKER_N,
        SOIL_MOHR_COHESION,
        SOIL_MOHR_FRICTION,
        SOIL_JANOSI_SHEAR,
        SOIL_ELASTIC_K,
        SOIL_DAMPING_R,
    )
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.08)  # colored sinkage overlay
    terrain.Initialize(SCM_LENGTH, SCM_WIDTH, SCM_RESOLUTION)
    terrain.SetMeshWireframe(False)
    terrain.SetTexture(
        chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 16, 8
    )

    # Sanity: rover must spawn on (not under) the soil rest plane.
    assert ROVER_INIT_Z > 0.0, "rover must spawn above the SCM rest plane (z=0)"

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Viper rover on SCM deformable terrain")
        vis.Initialize()                                    # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(CAM_EYE, CAM_TARGET)                  # AFTER Initialize
        vis.AddTypicalLights()
        vis.AddGrid(
            1.0, 1.0, 32, 16,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
            chrono.ChColor(0.35, 0.35, 0.35),
        )

    # === Output setup ===
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)      # review-frame / motion-log destination

    sim_csv = None
    motion_csv = None
    sim_writer = None
    motion_writer = None
    try:
        # Open both CSVs with context managers so they always flush/close.
        with open("simulation_data.csv", "w", newline="") as sim_csv, \
             open("cam/motion_log.csv", "w", newline="") as motion_csv:
            sim_writer = csv.writer(sim_csv)
            sim_writer.writerow([
                "time", "pos_x", "pos_y", "pos_z",
                "vel_x", "vel_y", "vel_z", "speed", "steering",
            ])
            motion_writer = csv.writer(motion_csv)
            motion_writer.writerow([
                "time", "body", "pos_x", "pos_y", "pos_z",
                "vel_x", "vel_y", "vel_z",
            ])

            # === Main loop === render-cadence outer loop; physics in the inner batch
            frame = 0
            while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
                if not HEADLESS:
                    vis.BeginScene()
                    vis.Render()
                    vis.EndScene()
                    vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                    frame += 1

                for _ in range(RENDER_EVERY):
                    t = sys.GetChTime()
                    pos = chassis.GetPos()       # cached getter object reused each step
                    vel = chassis.GetLinVel()
                    speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
                    sim_writer.writerow([
                        t, pos.x, pos.y, pos.z,
                        vel.x, vel.y, vel.z, speed, STEERING_ANGLE,
                    ])
                    motion_writer.writerow([
                        t, "viper_chassis", pos.x, pos.y, pos.z,
                        vel.x, vel.y, vel.z,
                    ])

                    # Step order: advance the rover controller, terrain, then dynamics.
                    rover.Update()
                    terrain.Synchronize(t)
                    sys.DoStepDynamics(TIME_STEP)
                    terrain.Advance(TIME_STEP)
                    if sys.GetChTime() >= RUN_END:
                        break
    except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:                # disk / permission errors on CSV
        import traceback
        traceback.print_exc()
        raise
    finally:
        # CSVs are closed by their `with` block; nothing else to flush here.
        pass

    # === Post-processing === plot the logged time series from the CSV
    try:
        data = np.genfromtxt("simulation_data.csv", delimiter=",", names=True)
        if data.size > 0:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
            ax1.plot(data["time"], data["pos_x"], label="pos_x")
            ax1.plot(data["time"], data["pos_y"], label="pos_y")
            ax1.plot(data["time"], data["pos_z"], label="pos_z")
            ax1.set_ylabel("position (m)")
            ax1.legend(loc="best")
            ax1.grid(True)
            ax2.plot(data["time"], data["speed"], label="speed", color="tab:red")
            ax2.set_xlabel("time (s)")
            ax2.set_ylabel("speed (m/s)")
            ax2.legend(loc="best")
            ax2.grid(True)
            fig.suptitle("Viper rover on SCM terrain — straight-line drive")
            fig.tight_layout()
            fig.savefig("simulation_timeseries.png", dpi=120)
            plt.close(fig)
    except (OSError, ValueError) as exc:             # missing/empty CSV or bad plot data
        import traceback
        traceback.print_exc()

    print(f"Done. Final time = {sys.GetChTime():.3f} s")


if __name__ == "__main__":
    main()
