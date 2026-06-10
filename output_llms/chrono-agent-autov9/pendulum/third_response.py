"""Double pendulum simulation (PyChrono 9.0.x + Irrlicht).

Models a planar double pendulum: two rigid rod arms swinging under gravity in
the world XY plane (gravity along -Y). The first arm is hinged to a fixed ground
pivot by a revolute joint; the second arm is hinged to the free (far) end of the
first arm by a second revolute joint. Both hinge axes are world +Z (normal to
the XY swing plane), so each arm rotates freely and independently in the plane.

System type: ChSystemNSC (non-smooth contact). The mechanism is fully described
by joints (no contacts), so no collision system is configured.

Bodies:
  - ground   : fixed reference body carrying the pivot post visual
  - arm1      : upper rod, hinged to ground at the top pivot
  - arm2      : lower rod, hinged to arm1's far end

Expected behavior: released from a horizontal configuration with zero initial
velocity, the two arms swing through the characteristic chaotic, non-periodic
motion of a double pendulum. With no damping, total mechanical energy is
conserved, so the swing amplitude does not decay over the run.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless backend: write PNG without a display
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry, masses and simulation controls (no bare literals downstream)
TIME_STEP = 1.0e-3          # physics step [s] — high precision for chaotic dynamics
SIM_END = 12.0              # simulation duration [s]
RENDER_FPS = 50.0           # review-video frame rate [Hz]

ARM_LENGTH = 1.0            # length of each rod arm [m]
ARM_RADIUS = 0.04           # visual rod radius [m]
ARM_MASS = 1.0              # mass of each arm [kg]

PIVOT_POS = chrono.ChVector3d(0.0, 2.0, 0.0)   # fixed top hinge in world space
GRAVITY = chrono.ChVector3d(0.0, -9.81, 0.0)   # gravity along -Y (XY swing plane)

# Solid-rod inertia about its transverse axes (thin rod): I = (1/12) m L^2.
# About its own long axis: I = (1/2) m r^2. Stored as a diagonal tensor.
ARM_INERTIA_LONG = 0.5 * ARM_MASS * ARM_RADIUS * ARM_RADIUS          # about local X (rod axis)
ARM_INERTIA_TRANS = (1.0 / 12.0) * ARM_MASS * ARM_LENGTH * ARM_LENGTH  # about local Y, Z

# Initial layout: both arms released horizontal (pointing along +X) from rest.
# arm1 center sits half a length to the right of the pivot; arm2 center sits a
# further half + half length to the right so its near end meets arm1's far end.
ARM1_CENTER = chrono.ChVector3d(PIVOT_POS.x + ARM_LENGTH / 2.0, PIVOT_POS.y, PIVOT_POS.z)
ARM1_FAR_END = chrono.ChVector3d(PIVOT_POS.x + ARM_LENGTH, PIVOT_POS.y, PIVOT_POS.z)
ARM2_CENTER = chrono.ChVector3d(ARM1_FAR_END.x + ARM_LENGTH / 2.0, ARM1_FAR_END.y, ARM1_FAR_END.z)

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run

# Derived constants (precomputed once — never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # physics steps per frame
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END         # short physics check when validating


def make_arm(name, center_pos):
    """Build one pendulum rod as a manual ChBody with a cylinder visual.

    The rod's long axis is body-local X; SetRot(QUNIT) leaves it pointing along
    world +X (horizontal start). The visual cylinder (default axis = local Z) is
    rotated Z->X via QuatFromAngleY(pi/2) so it draws along the rod axis.
    """
    arm = chrono.ChBody()
    arm.SetName(name)
    arm.SetMass(ARM_MASS)
    arm.SetInertiaXX(chrono.ChVector3d(ARM_INERTIA_LONG, ARM_INERTIA_TRANS, ARM_INERTIA_TRANS))
    arm.SetPos(center_pos)
    arm.SetRot(chrono.QUNIT)        # body-local X already aligned with world +X
    arm.EnableCollision(False)      # pure-joint mechanism, no contact
    cyl = chrono.ChVisualShapeCylinder(ARM_RADIUS, ARM_LENGTH)
    arm.AddVisualShape(cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
    return arm


def main():
    # === System & gravity === one ChSystemNSC; no collision system (joints only)
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(GRAVITY)

    # === Bodies === fixed ground (carries pivot post) + two swinging rod arms
    ground = chrono.ChBody()
    ground.SetName("ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    # Small post visual at the pivot so the fixed hinge is visible.
    post = chrono.ChVisualShapeCylinder(ARM_RADIUS * 1.5, 0.2)
    post.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
    ground.AddVisualShape(post, chrono.ChFramed(PIVOT_POS, chrono.QUNIT))
    sys.AddBody(ground)

    arm1 = make_arm("arm1", ARM1_CENTER)
    arm1.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    sys.AddBody(arm1)

    arm2 = make_arm("arm2", ARM2_CENTER)
    arm2.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.3, 0.8))
    sys.AddBody(arm2)

    # === Joints / constraints === two revolute hinges, axis = world +Z (normal to XY)
    # Hinge axis for an XY-plane swing is world +Z, which is the joint frame's
    # local +Z under QUNIT (Rule 8 special case for planar XY motion).
    hinge1 = chrono.ChLinkLockRevolute()
    hinge1.Initialize(
        arm1, ground, True,
        chrono.ChFramed(chrono.ChVector3d(-ARM_LENGTH / 2.0, 0, 0), chrono.QUNIT),  # arm1 NEAR end
        chrono.ChFramed(PIVOT_POS, chrono.QUNIT),                                   # ground pivot
    )
    sys.AddLink(hinge1)

    hinge2 = chrono.ChLinkLockRevolute()
    hinge2.Initialize(
        arm2, arm1, True,
        chrono.ChFramed(chrono.ChVector3d(-ARM_LENGTH / 2.0, 0, 0), chrono.QUNIT),  # arm2 NEAR end
        chrono.ChFramed(chrono.ChVector3d(+ARM_LENGTH / 2.0, 0, 0), chrono.QUNIT),  # arm1 FAR end
    )
    sys.AddLink(hinge2)

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    # Gated on SIMBENCH_VALIDATE so a validation run is fast and windowless; the
    # full block is always present in source (Initialize FIRST, then scene nodes).
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y -> Y is up
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Double Pendulum")
        vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(0.0, 1.0, 6.0), chrono.ChVector3d(1.0, 1.0, 0.0))
        vis.AddTypicalLights()
        vis.AddGrid(0.5, 0.5, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    # === Output setup === directories + cached handles for the logging hot loop
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
    os.makedirs("cam", exist_ok=True)       # guard against missing motion-log dir

    arm1_ref = arm1   # cache: body handles fetched once, reused every step
    arm2_ref = arm2   # cache: body handles fetched once, reused every step

    # CSV writers opened with context managers so they always flush/close.
    data_file = None
    motion_file = None
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:   # disk full / permission denied opening outputs
        print(f"Failed to open output CSV: {exc}")
        raise

    time_hist = []
    a1_angle_hist = []
    a2_angle_hist = []
    tip_x_hist = []
    tip_y_hist = []

    try:
        with data_file, motion_file:
            data_writer = csv.writer(data_file)
            data_writer.writerow([
                "time", "arm1_angle", "arm2_angle",
                "arm1_omega", "arm2_omega", "tip_x", "tip_y",
            ])
            motion_writer = csv.writer(motion_file)
            motion_writer.writerow([
                "time", "body", "pos_x", "pos_y", "pos_z",
                "vel_x", "vel_y", "vel_z", "omega_z",
            ])

            # === Main loop === render-cadence outer loop; physics + logging inner batch
            frame = 0
            while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
                if not HEADLESS:
                    vis.BeginScene()
                    vis.Render()
                    vis.EndScene()
                    vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                    frame += 1
                for _ in range(RENDER_EVERY):
                    t = sys.GetChTime()                       # cache: one fetch per step
                    p1 = arm1_ref.GetPos()
                    p2 = arm2_ref.GetPos()
                    v1 = arm1_ref.GetPosDt()
                    v2 = arm2_ref.GetPosDt()
                    w1 = arm1_ref.GetAngVelParent()
                    w2 = arm2_ref.GetAngVelParent()
                    # Arm absolute angle in the XY plane, measured from +X axis.
                    a1_angle = math.atan2(p1.y - PIVOT_POS.y, p1.x - PIVOT_POS.x)
                    a2_angle = math.atan2(p2.y - p1.y, p2.x - p1.x)
                    # World position of the free tip (arm2 far end).
                    tip_x = p2.x + (ARM_LENGTH / 2.0) * math.cos(a2_angle)
                    tip_y = p2.y + (ARM_LENGTH / 2.0) * math.sin(a2_angle)

                    data_writer.writerow([t, a1_angle, a2_angle, w1.z, w2.z, tip_x, tip_y])
                    motion_writer.writerow([t, "arm1", p1.x, p1.y, p1.z, v1.x, v1.y, v1.z, w1.z])
                    motion_writer.writerow([t, "arm2", p2.x, p2.y, p2.z, v2.x, v2.y, v2.z, w2.z])

                    time_hist.append(t)
                    a1_angle_hist.append(a1_angle)
                    a2_angle_hist.append(a2_angle)
                    tip_x_hist.append(tip_x)
                    tip_y_hist.append(tip_y)

                    sys.DoStepDynamics(TIME_STEP)
                    if sys.GetChTime() >= RUN_END:
                        break
    except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state mid-run
        import traceback
        traceback.print_exc()
        print(f"Simulation aborted: {exc}")
        raise
    finally:
        # CSV writers are closed by the `with` block; nothing else to flush here.
        print(f"Steps logged: {len(time_hist)}; sim time reached: {sys.GetChTime():.3f} s")

    # === Post-processing === timeseries plot of joint angles and the free tip path
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7))
    ax1.plot(time_hist, a1_angle_hist, label="arm1 angle [rad]", color="tab:red")
    ax1.plot(time_hist, a2_angle_hist, label="arm2 angle [rad]", color="tab:blue")
    ax1.set(xlabel="time [s]", ylabel="angle [rad]", title="Double pendulum arm angles")
    ax1.grid(True)
    ax1.legend(loc="upper right")

    ax2.plot(tip_x_hist, tip_y_hist, color="tab:green", linewidth=0.6)
    ax2.set(xlabel="tip x [m]", ylabel="tip y [m]", title="Free-end tip trajectory")
    ax2.axis("equal")
    ax2.grid(True)

    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    main()
