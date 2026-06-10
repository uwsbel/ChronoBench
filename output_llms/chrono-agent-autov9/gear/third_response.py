"""Gear train: motor-driven spur gear + 1:1 bevel gear + synchro-belt pulley.

Model (PyChrono 9.0.x, NSC rigid multibody)
-------------------------------------------
All wheels are mounted on a single fixed truss. Each wheel is a thin cylinder
(``ChBodyEasyCylinder`` with body-local Y as its geometric axis); the meshing is
imposed by ideal kinematic constraints (no tooth collision needed):

  * Gear A  - spur gear (radius 2), spun at a constant rate by a
              ``ChLinkMotorRotationSpeed`` between gear A and the truss. Its spin
              axis is world Z (body rotated 90 deg about X).
  * Gear D  - bevel gear (radius 5) at (-10, 0, -9), body rotated 90 deg about the
              world Z axis, hinged to the truss by a revolute about the horizontal
              (world X) axis. A ``ChLinkLockGear`` couples gear A and gear D with a
              1:1 transmission ratio (a bevel pairing - perpendicular shafts).
  * Pulley E- pulley (radius 2) at (-10, -11, -9), body rotated 90 deg about Z and
              hinged to the truss by a revolute about the horizontal (world X)
              axis. A ``ChLinkLockPulley`` models a synchro belt between gear D and
              pulley E, transmitting motion D -> E.

For every gear/pulley constraint the shaft ``ChFramed`` is rotated so its local Z
axis (taken as the wheel's spin axis) coincides with the cylinder's body-local Y
geometric axis - hence the ``QuatFromAngleX(-pi/2)`` shaft frames.

A simplified two-segment belt is drawn each frame between the belt tangent points
of pulley D and pulley E.

System type: ``ChSystemNSC``; integrator ``EULER_IMPLICIT_PROJECTED`` keeps the
gear/belt constraints stable. No contact/collision is configured because the
transmission is purely constraint-based.

Expected behavior
-----------------
The motor turns gear A at a constant speed. The 1:1 gear constraint makes gear D
turn at the same speed magnitude; the synchro belt then drives pulley E at a rate
scaled by the radius ratio R_D/R_E = 5/2 = 2.5. Motion is smooth and steady.
"""

# === Imports ===
import os
import csv
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

import matplotlib
matplotlib.use("Agg")            # headless plotting backend
import matplotlib.pyplot as plt

# === Named constants: geometry & physics ===
TIME_STEP = 1.0e-3               # integration step [s]
SIM_END = 6.0                    # total simulated time [s]
RENDER_FPS = 30.0                # review-video frame rate [Hz]

MOTOR_SPEED = 6.0                # gear A input angular speed [rad/s]

RADIUS_A = 2.0                   # spur gear A pitch radius [m]
RADIUS_D = 5.0                   # bevel gear D pitch radius [m]
RADIUS_E = 2.0                   # pulley E radius [m]
WHEEL_THICK = 0.6                # axial thickness of the wheel discs [m]
WHEEL_DENSITY = 1000.0           # disc material density [kg/m^3]

# Absolute wheel centers (gear A meshes with bevel gear D at perpendicular shafts).
POS_A = chrono.ChVector3d(0.0, 0.0, -1.0)            # spur gear A center
POS_D = chrono.ChVector3d(-10.0, 0.0, -9.0)          # bevel gear D center
POS_E = chrono.ChVector3d(-10.0, -11.0, -9.0)        # pulley E center

# Belt transmission ratio (pulley E speed / gear D speed) from the radii.
BELT_RATIO = RADIUS_D / RADIUS_E                      # expected |omega_E / omega_D|

# Rotation helpers (precomputed once).
ROT_BODY_X = chrono.QuatFromAngleX(chrono.CH_PI_2)   # body rot: spin axis Y -> world Z
ROT_BODY_Z = chrono.QuatFromAngleZ(chrono.CH_PI_2)   # body rot for D/E (90 deg about Z)
Q_REVOLUTE_HORIZ = chrono.QuatFromAngleY(chrono.CH_PI_2)   # revolute hinge -> world X axis
Q_SHAFT = chrono.QuatFromAngleX(-chrono.CH_PI_2)     # gear/pulley shaft: local Z onto body-Y

# Derived rendering cadence (precomputed once, never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # steps per frame

# Headless validation gate: fast, windowless physics check.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short check when validating


def make_wheel(radius, thickness, density, pos, rot, color):
    """Create a visible cylindrical wheel (geometric axis along body-local Y)."""
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radius, thickness,
                                     density, True, False)   # visualize, no collide
    body.SetPos(pos)
    body.SetRot(rot)
    body.GetVisualShape(0).SetColor(color)
    return body


def main():
    # === System & gravity ===
    sys = chrono.ChSystemNSC()                      # rigid NSC multibody system
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
    sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)
    # Pure constraint-driven mechanism: no contact, so no collision system is set.

    # === Bodies ===
    # Fixed truss carries every bearing (revolute hinge) of the mechanism.
    truss = chrono.ChBodyEasyBox(20, 10, 2, WHEEL_DENSITY, True, False)
    truss.SetFixed(True)
    truss.SetPos(chrono.ChVector3d(-5, -2, -5))
    truss.SetName("truss")
    truss.GetVisualShape(0).SetColor(chrono.ChColor(0.55, 0.55, 0.55))
    sys.Add(truss)

    # Spur gear A: spin axis world Z, motor-driven.
    gear_a = make_wheel(RADIUS_A, WHEEL_THICK, WHEEL_DENSITY,
                        POS_A, ROT_BODY_X, chrono.ChColor(0.2, 0.5, 0.9))
    gear_a.SetName("gear_A")
    sys.Add(gear_a)

    # Bevel gear D: body rotated 90 deg about Z -> horizontal (world X) spin axis.
    gear_d = make_wheel(RADIUS_D, WHEEL_THICK, WHEEL_DENSITY,
                        POS_D, ROT_BODY_Z, chrono.ChColor(0.9, 0.4, 0.2))
    gear_d.SetName("gear_D")
    sys.Add(gear_d)

    # Pulley E: same 90-deg-about-Z orientation, horizontal (world X) spin axis.
    pulley_e = make_wheel(RADIUS_E, WHEEL_THICK, WHEEL_DENSITY,
                          POS_E, ROT_BODY_Z, chrono.ChColor(0.3, 0.8, 0.3))
    pulley_e.SetName("pulley_E")
    sys.Add(pulley_e)

    # === Joints / constraints ===
    # Motor A<->truss: constant rotation speed about gear A's spin axis (frame Z).
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(gear_a, truss, chrono.ChFramed(POS_A, chrono.QUNIT))
    motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
    sys.AddLink(motor)

    # Revolute D<->truss about the horizontal world X axis (hinge frame Z -> X).
    rev_d = chrono.ChLinkLockRevolute()
    rev_d.Initialize(gear_d, truss, chrono.ChFramed(POS_D, Q_REVOLUTE_HORIZ))
    sys.AddLink(rev_d)

    # Revolute E<->truss about the horizontal world X axis (hinge frame Z -> X).
    rev_e = chrono.ChLinkLockRevolute()
    rev_e.Initialize(pulley_e, truss, chrono.ChFramed(POS_E, Q_REVOLUTE_HORIZ))
    sys.AddLink(rev_e)

    # Gear constraint A<->D, 1:1 ratio (bevel pairing). The shaft frames map each
    # constraint's local Z onto the wheel's body-local Y geometric axis; only the
    # shaft positions/axes and the ratio are needed for the bevel case.
    gear_ad = chrono.ChLinkLockGear()
    gear_ad.Initialize(gear_a, gear_d, chrono.ChFramed())
    gear_ad.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, Q_SHAFT))
    gear_ad.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, Q_SHAFT))
    gear_ad.SetTransmissionRatio(1.0)               # 1:1 between gear A and gear D
    sys.AddLink(gear_ad)

    # Synchro-belt constraint D<->E. The two shafts are parallel (both world X);
    # the radii set the belt transmission. EnforcePhase prevents belt slipping.
    belt = chrono.ChLinkLockPulley()
    belt.Initialize(gear_d, pulley_e, chrono.ChFramed())
    belt.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, Q_SHAFT))
    belt.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, Q_SHAFT))
    belt.SetRadius1(RADIUS_D)
    belt.SetRadius2(RADIUS_E)
    belt.SetEnforcePhase(True)                      # synchro belt: no slip
    sys.AddLink(belt)

    # === Visualization (full Irrlicht scene) ===
    # Gated by HEADLESS for fast validation; the full window block is present below.
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Gear train: bevel gear D + synchro-belt pulley E")
        vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
        vis.Initialize()                                    # Initialize FIRST (Irrlicht)
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(12, 15, -20), chrono.ChVector3d(-6, -4, -6))
        vis.AddTypicalLights()
        vis.AddGrid(1.0, 1.0, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(-5, -7, -5), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    # === Output setup ===
    os.makedirs("frames", exist_ok=True)   # guard against missing frame dir
    os.makedirs("cam", exist_ok=True)       # guard against missing cam dir

    # cache: getters fetched once, reused every step (efficiency)
    get_time = sys.GetChTime                 # cache: bound method, called per step
    rot_a = gear_ad.GetRotation1             # cache: gear A absolute rotation getter
    rot_d = gear_ad.GetRotation2             # cache: gear D absolute rotation getter
    rot_e = belt.GetRotation2                # cache: pulley E absolute rotation getter

    data_f = None
    motion_f = None
    try:
        # Guard file opens specifically (disk / permission failures).
        try:
            data_f = open("simulation_data.csv", "w", newline="")
            motion_f = open("cam/motion_log.csv", "w", newline="")
        except (OSError, IOError) as exc:    # disk full / permission denied
            print("Failed to open CSV output:", exc)
            raise

        with data_f, motion_f:
            data_writer = csv.writer(data_f)
            data_writer.writerow(["time", "omega_A", "omega_D", "omega_E",
                                  "rot_A", "rot_D", "rot_E"])
            motion_writer = csv.writer(motion_f)
            motion_writer.writerow([
                "time",
                "A_x", "A_y", "A_z", "A_wx", "A_wy", "A_wz",
                "D_x", "D_y", "D_z", "D_wx", "D_wy", "D_wz",
                "E_x", "E_y", "E_z", "E_wx", "E_wy", "E_wz",
            ])

            # === Main loop (render-cadence outer loop) ===
            frame = 0
            while (HEADLESS or vis.Run()) and get_time() < RUN_END:
                if not HEADLESS:
                    vis.BeginScene()
                    vis.Render()
                    # Simplified two-segment belt between the D and E tangent points.
                    chronoirr.drawSegment(vis, belt.GetBeltUpPos1(), belt.GetBeltUpPos2())
                    chronoirr.drawSegment(vis, belt.GetBeltBottomPos1(), belt.GetBeltBottomPos2())
                    vis.EndScene()
                    vis.WriteImageToFile(f"frames/img_{frame:06d}.png")
                    frame += 1

                for _ in range(RENDER_EVERY):
                    t = get_time()
                    wa = gear_a.GetAngVelParent()    # gear A spins about world Z
                    wd = gear_d.GetAngVelParent()    # gear D spins about world X
                    we = pulley_e.GetAngVelParent()  # pulley E spins about world X
                    data_writer.writerow([
                        t, wa.z, wd.x, we.x,
                        rot_a(), rot_d(), rot_e(),
                    ])
                    pa, pd, pe = gear_a.GetPos(), gear_d.GetPos(), pulley_e.GetPos()
                    motion_writer.writerow([
                        t,
                        pa.x, pa.y, pa.z, wa.x, wa.y, wa.z,
                        pd.x, pd.y, pd.z, wd.x, wd.y, wd.z,
                        pe.x, pe.y, pe.z, we.x, we.y, we.z,
                    ])
                    sys.DoStepDynamics(TIME_STEP)
                    if get_time() >= RUN_END:
                        break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # CSV writers are closed by the `with` block; report completion here.
        print(f"Simulation reached t = {sys.GetChTime():.3f} s")

    # === Post-processing: plot the logged angular speeds ===
    times, oa, od, oe = [], [], [], []
    try:
        with open("simulation_data.csv", "r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row["time"]))
                oa.append(float(row["omega_A"]))
                od.append(float(row["omega_D"]))
                oe.append(float(row["omega_E"]))
    except (OSError, IOError) as exc:           # missing/locked CSV on read-back
        print("Could not read CSV for plotting:", exc)
        return

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(times, oa, label="gear A (omega_z)")
    ax.plot(times, od, label="gear D (omega_x)")
    ax.plot(times, oe, label="pulley E (omega_x)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("angular speed [rad/s]")
    ax.set_title("Gear train angular speeds (A -> D -> belt -> E)")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)
    print("Wrote simulation_timeseries.png")


if __name__ == "__main__":
    main()
