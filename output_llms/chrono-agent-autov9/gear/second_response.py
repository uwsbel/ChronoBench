"""
Spur-gear pair driven by a constant-speed rotational motor (PyChrono 9.0.1, NSC).

Model
-----
Two cylindrical gear wheels (gear A and gear B) spin about parallel vertical
(world Y) shafts that are pinned to a fixed truss/ground body by revolute joints.
A ChLinkLockGear kinematic constraint couples the two shafts so their angular
velocities respect the gear ratio radA/radB (gear A drives gear B). A
ChLinkMotorRotationSpeed imposes a constant angular speed on gear A about its
shaft; the gear constraint then enforces gear B's matching counter-rotation.

System type : ChSystemNSC (rigid bodies, kinematic joints, no contact needed).
Main bodies : truss (fixed support), gear A (radius 1.5), gear B (radius 3.5),
              plus a thin visual drive shaft on gear A.
Expected    : gear A turns at the commanded constant speed (3 rad/s); gear B
              turns in the opposite sense at speed scaled by radA/radB, with a
              steady (non-diverging) transmission ratio and smooth motion.

All output (frames, CSV, plot) is written next to this script.
"""

import os
import csv
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# === Named constants === geometry / physics / run parameters (no bare literals downstream)
TIME_STEP = 1.0e-3          # integration step [s]
SIM_END = 6.0               # total simulated time [s]
RENDER_FPS = 30.0           # review-video frame rate [frames/s]

RAD_A = 1.5                 # pitch radius of gear A [m]
RAD_B = 3.5                 # pitch radius of gear B [m]
GEAR_THICKNESS = 0.5        # axial thickness of each gear wheel [m]
GEAR_DENSITY = 1000.0       # material density of the gears [kg/m^3]

TRUSS_SX = 15.0             # truss box width  (X) [m]
TRUSS_SY = 8.0              # truss box height (Y) [m]
TRUSS_SZ = 2.0             # truss box depth  (Z) [m]
TRUSS_DENSITY = 1000.0      # truss material density [kg/m^3]

MOTOR_SPEED = 3.0           # commanded constant angular speed of gear A [rad/s]

SHAFT_RADIUS = RAD_A * 0.3  # visual drive-shaft radius [m]
SHAFT_LENGTH = 10.0         # visual drive-shaft length [m]

INTERAXIS_12 = RAD_A + RAD_B            # center distance between the two shafts [m]  (precomputed once)
GEAR_A_POS = chrono.ChVector3d(0, 0, -1)              # gear A center [m]
GEAR_B_POS = chrono.ChVector3d(INTERAXIS_12, 0, -2)   # gear B center [m]
SHAFT_AXIS = chrono.ChVector3d(0, 1, 0)               # shafts spin about world +Y

# Render cadence: advance this many physics steps between rendered frames (precomputed once).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Fast, windowless validation run when SIMBENCH_VALIDATE is set (speed only, not concurrency).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))


def build_gear(sys, radius, pos, density, color, mat):
    """Create one cylindrical gear wheel spinning about world Y, placed at pos."""
    # ChBodyEasyCylinder(axis, radius, height, density, visualize, collide, material)
    gear = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radius, GEAR_THICKNESS,
                                     density, True, False, mat)
    gear.SetPos(pos)
    gear.GetVisualShape(0).SetColor(color)
    sys.Add(gear)
    return gear


def main():
    # === System & gravity === rigid multibody, gravity along -Y; no contact -> no collision system
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    contact_mat = chrono.ChContactMaterialNSC()   # required by easy-body factory signature
    contact_mat.SetFriction(0.3)

    # === Bodies === fixed truss support + two gear wheels + a visual drive shaft on gear A
    truss = chrono.ChBodyEasyBox(TRUSS_SX, TRUSS_SY, TRUSS_SZ,
                                 TRUSS_DENSITY, True, False, contact_mat)
    truss.SetPos(chrono.ChVector3d(0, 0, 0))
    truss.SetFixed(True)
    truss.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    sys.Add(truss)

    gear_a = build_gear(sys, RAD_A, GEAR_A_POS, GEAR_DENSITY,
                        chrono.ChColor(0.9, 0.4, 0.2), contact_mat)
    gear_b = build_gear(sys, RAD_B, GEAR_B_POS, GEAR_DENSITY,
                        chrono.ChColor(0.2, 0.5, 0.9), contact_mat)

    # Visual drive shaft attached to gear A (structural, axial along the shaft / world Y).
    shaft_vis = chrono.ChVisualShapeCylinder(SHAFT_RADIUS, SHAFT_LENGTH)
    shaft_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
    gear_a.AddVisualShape(shaft_vis)   # cylinder default axis = body-local Y here (easy-cyl body)

    # === Joints / constraints === revolutes to ground, gear coupling, constant-speed motor
    # Gear A revolute about its shaft (frame local +Z must map to the world Y shaft axis).
    q_y = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.ChVector3d(1, 0, 0))  # +Z -> +Y
    rev_a = chrono.ChLinkLockRevolute()
    rev_a.Initialize(gear_a, truss, chrono.ChFramed(GEAR_A_POS, q_y))
    sys.AddLink(rev_a)

    rev_b = chrono.ChLinkLockRevolute()
    rev_b.Initialize(gear_b, truss, chrono.ChFramed(GEAR_B_POS, q_y))
    sys.AddLink(rev_b)

    # Gear kinematic constraint. Initialize with an ABSOLUTE meshing frame (local +Z = shaft
    # axis = world +Y), then set each shaft's absolute frame so the link knows both rotation
    # axes and the center distance. Ratio radA:radB -> coupled angular speeds, opposite sense.
    link_gear = chrono.ChLinkLockGear()
    link_gear.Initialize(gear_a, gear_b, chrono.ChFramed(GEAR_A_POS, q_y))
    # Shaft frames are BODY-LOCAL: at each gear's own origin, local +Z mapped to its world +Y axis.
    link_gear.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, q_y))
    link_gear.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, q_y))
    link_gear.SetTransmissionRatio(RAD_A, RAD_B)
    link_gear.SetEnforcePhase(False)   # speed coupling only, no absolute phase lock
    link_gear.SetEpicyclic(False)      # ordinary (non-planetary) gear pair
    sys.AddLink(link_gear)

    # Constant-speed motor driving gear A about its shaft relative to the fixed truss.
    link_motor = chrono.ChLinkMotorRotationSpeed()
    link_motor.Initialize(gear_a, truss, chrono.ChFramed(GEAR_A_POS, q_y))
    link_motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
    sys.AddLink(link_motor)

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Spur Gear Pair - constant-speed drive")
        vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y
        vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(8, 8, -12), chrono.ChVector3d(INTERAXIS_12 * 0.5, 0, -1.5))
        vis.AddTypicalLights()
        vis.AddGrid(0.5, 0.5, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, -4.0, 0),
                                       chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.ChVector3d(1, 0, 0))),
                    chrono.ChColor(0.4, 0.4, 0.4))

    # === Main loop === render-cadence outer loop + per-step physics batch with CSV logging
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

    # cache: link handles fetched once, reused every step (avoid repeated lookups in hot loop)
    motor = link_motor
    gear = link_gear

    times, ang_a, ang_b, spd_a, spd_b, ratio = [], [], [], [], [], []

    sim_csv = None
    motion_csv = None
    try:
        sim_csv = open("simulation_data.csv", "w", newline="")          # noqa: SIM115 (closed in finally)
        motion_csv = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
        sim_writer = csv.writer(sim_csv)
        motion_writer = csv.writer(motion_csv)
        sim_writer.writerow(["time", "motor_angle", "motor_speed",
                             "gearA_rot", "gearB_rot", "transmission_ratio"])
        motion_writer.writerow(["time", "body", "x", "y", "z", "wx", "wy", "wz"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1
            for _ in range(RENDER_EVERY):
                t = sys.GetChTime()                 # cache: single time read per step
                m_ang = motor.GetMotorAngle()
                m_spd = motor.GetMotorAngleDt()
                rot_a = gear.GetRotation1()
                rot_b = gear.GetRotation2()
                tr = gear.GetTransmissionRatio()

                sim_writer.writerow([t, m_ang, m_spd, rot_a, rot_b, tr])
                wa = gear_a.GetAngVelParent()
                wb = gear_b.GetAngVelParent()
                pa = gear_a.GetPos()
                pb = gear_b.GetPos()
                motion_writer.writerow([t, "gearA", pa.x, pa.y, pa.z, wa.x, wa.y, wa.z])
                motion_writer.writerow([t, "gearB", pb.x, pb.y, pb.z, wb.x, wb.y, wb.z])

                times.append(t)
                ang_a.append(rot_a)
                ang_b.append(rot_b)
                spd_a.append(wa.y)
                spd_b.append(wb.y)
                ratio.append(tr)

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= run_end:
                    break
    except (OSError, IOError) as exc:               # disk / permission error on the CSV files
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:       # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush + close any open writers even if a step diverged mid-run
        if sim_csv is not None:
            sim_csv.close()
        if motion_csv is not None:
            motion_csv.close()

    # === Post-processing === plot gear rotations / speeds / ratio vs time from logged arrays
    try:
        fig, (axr, axs, axt) = plt.subplots(3, sharex=True, figsize=(9, 8))
        axr.plot(times, ang_a, label="gear A rotation [rad]")
        axr.plot(times, ang_b, label="gear B rotation [rad]")
        axr.set(ylabel="rotation [rad]")
        axr.grid(True)
        axr.legend(loc="upper left")

        axs.plot(times, spd_a, label="gear A wY [rad/s]")
        axs.plot(times, spd_b, label="gear B wY [rad/s]")
        axs.set(ylabel="ang. velocity [rad/s]")
        axs.grid(True)
        axs.legend(loc="upper left")

        axt.plot(times, ratio, color="purple", label="transmission ratio")
        axt.set(ylabel="ratio [-]", xlabel="time [s]")
        axt.grid(True)
        axt.legend(loc="upper left")

        fig.suptitle("Spur gear pair: motor speed 3 rad/s, radA=1.5, radB=3.5")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)
    except (ValueError, RuntimeError) as exc:        # empty data / backend plotting failure
        import traceback
        traceback.print_exc()

    print(f"Done. steps logged={len(times)}  final gearA rot={ang_a[-1]:.3f} rad"
          f"  final gearB rot={ang_b[-1]:.3f} rad")


if __name__ == "__main__":
    main()
