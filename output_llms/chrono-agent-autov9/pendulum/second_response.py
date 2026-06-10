"""Single spherical pendulum swinging under lunar gravity (PyChrono 9.0.x, NSC).

Model
-----
A rigid rod-shaped pendulum arm hangs from a fixed pivot anchored to a static
ground body. The arm is attached to the pivot by a SPHERICAL joint, so the arm
is free to swing in all three rotational degrees of freedom about the pivot
point while the pivot point itself stays fixed in space.

System / physics
----------------
* ChSystemNSC (non-smooth contact). The mechanism is joint-only (no contacts),
  so no collision system or contact material is created.
* Gravity is set to the Moon's surface value (0, -1.62, 0) m/s^2 — gravity acts
  along world -Y, hence a Y-up vertical convention for the camera.
* Pendulum arm: mass 2 kg, principal inertia tensor (0.4, 1.5, 1.5) kg*m^2,
  visualized as a cylinder of radius 0.1 m and length (height) 1.5 m.
* The pivot is visualized as a sphere of radius 2 m attached to the ground body.
* The arm receives a non-zero initial angular velocity so it starts swinging
  immediately rather than hanging perfectly still.

Expected behavior
------------------
Under the weak lunar gravity and the imposed initial spin, the pendulum performs
a slow, smooth three-dimensional swing about the fixed pivot. With no damping and
no contact, the motion is conservative — total mechanical energy stays bounded
and the pivot-to-tip distance is held constant by the spherical constraint.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for PNG timeseries output
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / run control) ===
TIME_STEP = 1.0e-3          # physics step [s] — high-precision MBS
SIM_END = 10.0              # total simulated duration [s]
RENDER_FPS = 50.0           # review-video frame rate [frames/s]

MOON_GRAVITY = chrono.ChVector3d(0.0, -1.62, 0.0)  # lunar surface gravity [m/s^2]

ARM_MASS = 2.0                                      # pendulum mass [kg]
ARM_INERTIA = chrono.ChVector3d(0.4, 1.5, 1.5)      # principal inertia tensor [kg*m^2]
ARM_RADIUS = 0.1                                     # arm cylinder radius [m]
ARM_LENGTH = 1.5                                     # arm cylinder height / length [m]

PIVOT_SPHERE_RADIUS = 2.0                            # joint marker sphere radius [m]

# Initial angular velocity of the arm about world Z [rad/s] (kick-starts the swing).
INIT_ANG_VEL = chrono.ChVector3d(0.0, 0.0, 1.5)

# === Derived geometry (precomputed once) ===
PIVOT_POS = chrono.ChVector3d(0.0, 0.0, 0.0)         # fixed pivot at world origin
# Arm origin at its own center; it hangs straight down, so its COM sits a
# half-length below the pivot. Far (tip) end is then a full length below pivot.
ARM_CENTER = chrono.ChVector3d(0.0, PIVOT_POS.y - ARM_LENGTH / 2.0, 0.0)
ARM_TIP_LOCAL = chrono.ChVector3d(0.0, -ARM_LENGTH / 2.0, 0.0)  # tip in arm-local frame
# Spherical joint attaches at the arm's TOP end (local +half-length along arm axis).
ARM_TOP_LOCAL = chrono.ChVector3d(0.0, ARM_LENGTH / 2.0, 0.0)

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # physics steps per frame
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating

# === System & gravity === build the NSC system and apply lunar gravity
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(MOON_GRAVITY)  # simulate the pendulum on the Moon
# Joint-only mechanism: no collision system / contact material needed.

# === Bodies === fixed ground (carrying the pivot sphere) + the pendulum arm
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(PIVOT_POS)
sys.AddBody(ground)

# Pivot is rendered as a sphere of radius 2 attached to the ground at the pivot.
# The arm (length 1.5) swings inside this large sphere, so the sphere is made
# semi-transparent — the requested radius is preserved while the arm stays visible.
pivot_sphere = chrono.ChVisualShapeSphere(PIVOT_SPHERE_RADIUS)
pivot_sphere.SetColor(chrono.ChColor(0.2, 0.5, 0.9))
pivot_sphere.SetOpacity(0.25)  # see the swinging arm through the large pivot marker
ground.AddVisualShape(pivot_sphere, chrono.ChFramed(PIVOT_POS, chrono.QUNIT))

# Pendulum arm: manual ChBody so mass + inertia tensor are set exactly.
arm = chrono.ChBody()
arm.SetMass(ARM_MASS)
arm.SetInertiaXX(ARM_INERTIA)
arm.SetPos(ARM_CENTER)
arm.EnableCollision(False)  # pure joint mechanism, no contact
sys.AddBody(arm)

# Arm visual: a cylinder of radius 0.1 and height 1.5 along the body-local Y axis.
# ChVisualShapeCylinder's default axis is body-local Z, so rotate Z -> Y for a
# vertical hanging rod (body-local Y is the arm axis here).
arm_cyl = chrono.ChVisualShapeCylinder(ARM_RADIUS, ARM_LENGTH)
arm_cyl.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
arm.AddVisualShape(arm_cyl, chrono.ChFramed(chrono.VNULL,
                                            chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Small tip marker so the swinging end is clearly visible in the review video.
tip_marker = chrono.ChVisualShapeSphere(ARM_RADIUS * 1.5)
tip_marker.SetColor(chrono.ChColor(0.95, 0.85, 0.1))
arm.AddVisualShape(tip_marker, chrono.ChFramed(ARM_TIP_LOCAL, chrono.QUNIT))

# Initial angular velocity (world frame) so the arm begins swinging at t=0.
arm.SetAngVelParent(INIT_ANG_VEL)

# === Joints / constraints === spherical joint pins the arm's top end to ground
# A spherical joint allows all 3 rotational DOF about the pivot while holding the
# attachment point fixed. Use the 5-arg (explicit relative-frame) form so the
# attachment is the arm's TOP end, not its center.
spherical = chrono.ChLinkLockSpherical()
spherical.Initialize(
    arm, ground, True,
    chrono.ChFramed(ARM_TOP_LOCAL, chrono.QUNIT),  # arm local: top end
    chrono.ChFramed(PIVOT_POS, chrono.QUNIT),      # ground local: pivot point
)
sys.AddLink(spherical)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # gravity along -Y -> Y is up
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Spherical Pendulum on the Moon")
    vis.Initialize()  # Initialize FIRST, then add scene elements (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    # Camera framed so the full radius-2 pivot sphere AND the swinging 1.5 m arm
    # fit in one continuous view (sphere spans 4 m across), looking at the pivot.
    vis.AddCamera(chrono.ChVector3d(7.0, 0.5, 7.0), chrono.ChVector3d(0.0, -1.0, 0.0))
    vis.AddTypicalLights()
    # Lay the grid flat in the horizontal world XZ plane (rotate its local XY plane
    # 90 deg about X) and drop it below the sphere as a clean ground reference.
    vis.AddGrid(0.5, 0.5, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, -(PIVOT_SPHERE_RADIUS + 0.5), 0),
                                   chrono.QuatFromAngleX(chrono.CH_PI_2)),
                chrono.ChColor(0.4, 0.4, 0.4))  # horizontal ground grid below the sphere

# === Main loop === render-cadence outer loop; physics + CSV logging in inner batch
os.makedirs("frames", exist_ok=True)  # guard against missing frame output dir
os.makedirs("cam", exist_ok=True)     # guard against missing motion-log dir

# Cache the constant pendulum length once for the constraint-check column.
pendulum_length = ARM_LENGTH  # cache: constant arm length, reused every step

# Logging buffers for the timeseries plot (filled every physics step).
log_t = []
log_tip_x = []
log_tip_y = []
log_tip_z = []
log_speed = []

data_file = None
motion_file = None
try:
    # Guard the file opens specifically (disk / permission errors).
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:  # disk full / permission denied
        print(f"Could not open output CSV files: {exc}")
        raise

    data_writer = csv.writer(data_file)
    data_writer.writerow(["time", "tip_x", "tip_y", "tip_z",
                          "arm_speed", "arm_omega_z", "tip_radius"])
    motion_writer = csv.writer(motion_file)
    motion_writer.writerow(["time", "body", "pos_x", "pos_y", "pos_z",
                            "vel_x", "vel_y", "vel_z"])

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
            frame += 1
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()                 # cache: current sim time, reused below
            arm_pos = arm.GetPos()              # cache: COM pose this step
            arm_vel = arm.GetPosDt()            # cache: COM velocity this step
            arm_omega = arm.GetAngVelParent()   # cache: angular velocity this step
            # Tip (free end) world position from the constant body-local offset.
            tip = arm.TransformPointLocalToParent(ARM_TIP_LOCAL)
            speed = arm_vel.Length()
            tip_radius = (tip - PIVOT_POS).Length()  # spherical-constraint check

            data_writer.writerow([t, tip.x, tip.y, tip.z,
                                  speed, arm_omega.z, tip_radius])
            motion_writer.writerow([t, "arm", arm_pos.x, arm_pos.y, arm_pos.z,
                                    arm_vel.x, arm_vel.y, arm_vel.z])

            log_t.append(t)
            log_tip_x.append(tip.x)
            log_tip_y.append(tip.y)
            log_tip_z.append(tip.z)
            log_speed.append(speed)

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= RUN_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close any open writers even if a step diverges mid-run.
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing === timeseries PNG from the logged buffers
if log_t:
    t_arr = np.array(log_t)
    fig, (ax1, ax2) = plt.subplots(2, sharex=True, figsize=(9, 7))
    ax1.plot(t_arr, np.array(log_tip_x), label="tip x")
    ax1.plot(t_arr, np.array(log_tip_y), label="tip y")
    ax1.plot(t_arr, np.array(log_tip_z), label="tip z")
    ax1.set(ylabel="tip position [m]", title="Spherical pendulum on the Moon")
    ax1.grid()
    ax1.legend(loc="upper right")

    ax2.plot(t_arr, np.array(log_speed), "r-")
    ax2.set(ylabel="COM speed [m/s]", xlabel="time [s]")
    ax2.grid()

    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

print(f"Done: {len(log_t)} steps logged, length constraint target = {pendulum_length:.3f} m")
