"""Crank-slider mechanism driven at a constant crank speed (PyChrono 9.0.1 + Irrlicht).

Model
-----
A classic planar crank-slider (single-cylinder engine kinematics) built as a pure
multi-body system (ChSystemNSC, no collision/contact — the mechanism is fully
defined by joints and a motor):

  * truss   : fixed ground body holding the crank pivot and the slider guide.
  * crank   : a short link rotating about the fixed pivot at the world origin.
  * rod     : the connecting rod joining the crank pin to the piston pin.
  * piston  : the slider, constrained to translate along the world X guide.

Topology (planar XY motion, gravity along -Y, all hinge axes along world +Z):
  truss--crank   revolute  (ground pivot at origin) + rotation-speed MOTOR
  crank--rod     revolute  (crank pin)
  rod--piston    revolute  (wrist pin)
  piston--truss  prismatic (fixed X guide)

Expected behavior
-----------------
The motor spins the crank at a constant angular speed, so the crank angle grows
linearly in time. The piston oscillates back and forth along X between
(crank_len + rod_len) and (rod_len - crank_len) with a period equal to one crank
revolution; the rod swings, and the piston velocity is roughly sinusoidal. CSV
columns and the time-series PNG capture this periodic slider motion.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for PNG output
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants (geometry, physics, run control) ===
TIME_STEP = 1.0e-3          # physics step [s] — high-precision mechanism
SIM_END = 6.0               # simulated duration [s]
RENDER_FPS = 50.0           # review-frame cadence [frames/s]

CRANK_LEN = 1.0             # crank pin radius from pivot [m]
ROD_LEN = 3.0               # connecting-rod length [m]
CRANK_SPEED = 2.0 * math.pi # constant crank angular speed [rad/s] -> 1 rev/s

CRANK_RADIUS = 0.07         # crank link cylinder radius [m] (visual)
ROD_RADIUS = 0.05           # rod cylinder radius [m] (visual)
PISTON_SX = 0.4             # piston box size along X [m]
PISTON_SY = 0.4             # piston box size along Y/Z [m]

CRANK_MASS = 2.0            # crank mass [kg]
ROD_MASS = 1.0              # rod mass [kg]
PISTON_MASS = 1.5           # piston mass [kg]

GUIDE_Y = 0.0               # the slider guide lies on the X axis (y = 0)
GUIDE_Z = 0.0

# Derived initial geometry (crank horizontal at t=0, pin on +X) — precomputed once.
PIVOT_POS = chrono.ChVector3d(0.0, 0.0, 0.0)             # ground pivot at origin
CRANK_PIN0 = chrono.ChVector3d(CRANK_LEN, 0.0, 0.0)       # crank pin world pos at t=0
# Piston starts on the guide so the rod just spans crank pin -> piston.
PISTON_X0 = CRANK_LEN + ROD_LEN
PISTON_POS0 = chrono.ChVector3d(PISTON_X0, GUIDE_Y, GUIDE_Z)
CRANK_CENTER0 = chrono.ChVector3d(CRANK_LEN * 0.5, 0.0, 0.0)   # crank COM (mid link)
ROD_CENTER0 = chrono.ChVector3d((CRANK_LEN + PISTON_X0) * 0.5, 0.0, 0.0)  # rod COM (mid)

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless check


def main():
    # === System & gravity === pure MBS, no collision system (joints + motor only)
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))

    # === Bodies === truss (ground), crank, connecting rod, piston (slider)
    # Truss: fixed ground carrying the pivot and the slider guide.
    truss = chrono.ChBody()
    truss.SetFixed(True)
    truss.SetName("truss")
    sys.AddBody(truss)
    # Visualize the fixed pivot bearing so the support structure is not invisible.
    pivot_vis = chrono.ChVisualShapeCylinder(CRANK_RADIUS * 1.4, 0.30)
    pivot_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
    truss.AddVisualShape(pivot_vis, chrono.ChFramed(PIVOT_POS, chrono.QUNIT))

    # Crank: rotates about world +Z; body-local X aligned with world X at t=0.
    crank = chrono.ChBody()
    crank.SetMass(CRANK_MASS)
    crank.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
    crank.SetPos(CRANK_CENTER0)
    crank.SetRot(chrono.QUNIT)        # local X = world X (crank horizontal at start)
    crank.EnableCollision(False)
    sys.AddBody(crank)
    crank_vis = chrono.ChVisualShapeCylinder(CRANK_RADIUS, CRANK_LEN)
    crank_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    # Step-2 visual offset: rotate cylinder local Z -> local X (link axis).
    crank.AddVisualShape(crank_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

    # Connecting rod: spans crank pin -> piston pin; body-local X along world X at start.
    rod = chrono.ChBody()
    rod.SetMass(ROD_MASS)
    rod.SetInertiaXX(chrono.ChVector3d(0.01, 0.5, 0.5))
    rod.SetPos(ROD_CENTER0)
    rod.SetRot(chrono.QUNIT)
    rod.EnableCollision(False)
    sys.AddBody(rod)
    rod_vis = chrono.ChVisualShapeCylinder(ROD_RADIUS, ROD_LEN)
    rod_vis.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
    rod.AddVisualShape(rod_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

    # Piston (slider): translates along the world X guide.
    piston = chrono.ChBody()
    piston.SetMass(PISTON_MASS)
    piston.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
    piston.SetPos(PISTON_POS0)
    piston.SetRot(chrono.QUNIT)
    piston.EnableCollision(False)
    sys.AddBody(piston)
    piston_vis = chrono.ChVisualShapeBox(PISTON_SX, PISTON_SY, PISTON_SY)
    piston_vis.SetColor(chrono.ChColor(0.2, 0.7, 0.2))
    piston.AddVisualShape(piston_vis)

    # === Joints / constraints === ground revolute+motor, two pin revolutes, slider prismatic
    # truss--crank revolute at the fixed pivot; planar XY motion -> hinge about world +Z (QUNIT).
    rev_truss_crank = chrono.ChLinkLockRevolute()
    rev_truss_crank.Initialize(crank, truss, chrono.ChFramed(PIVOT_POS, chrono.QUNIT))
    sys.AddLink(rev_truss_crank)

    # Constant-speed motor on the same ground pivot (geometric hinge above stays).
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(crank, truss, chrono.ChFramed(PIVOT_POS, chrono.QUNIT))
    motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))  # constant angular speed
    sys.AddLink(motor)

    # crank--rod revolute at the crank pin (world +Z hinge). Body-local frames:
    # crank far end is local +CRANK_LEN/2; rod near end is local -ROD_LEN/2.
    rev_crank_rod = chrono.ChLinkLockRevolute()
    rev_crank_rod.Initialize(
        crank, rod, True,
        chrono.ChFramed(chrono.ChVector3d(CRANK_LEN * 0.5, 0.0, 0.0), chrono.QUNIT),
        chrono.ChFramed(chrono.ChVector3d(-ROD_LEN * 0.5, 0.0, 0.0), chrono.QUNIT),
    )
    sys.AddLink(rev_crank_rod)

    # rod--piston revolute (wrist pin). Rod far end local +ROD_LEN/2 -> piston center.
    rev_rod_piston = chrono.ChLinkLockRevolute()
    rev_rod_piston.Initialize(
        rod, piston, True,
        chrono.ChFramed(chrono.ChVector3d(ROD_LEN * 0.5, 0.0, 0.0), chrono.QUNIT),
        chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
    )
    sys.AddLink(rev_rod_piston)

    # piston--truss prismatic along world X. Prismatic frame local +Z must map to the
    # guide axis (X) -> Q_ROTATE_Z_TO_X.
    prism_piston_truss = chrono.ChLinkLockPrismatic()
    prism_piston_truss.Initialize(
        piston, truss, chrono.ChFramed(PISTON_POS0, chrono.Q_ROTATE_Z_TO_X)
    )
    sys.AddLink(prism_piston_truss)

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    # Gated on SIMBENCH_VALIDATE for a fast, windowless physics check (full block below).
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y -> Y is up
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Crank-Slider Mechanism (constant crank speed)")
        vis.Initialize()                                    # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()                                     # standard sky backdrop
        vis.AddCamera(chrono.ChVector3d(2.0, 2.5, 6.0), chrono.ChVector3d(2.0, 0.0, 0.0))
        vis.AddTypicalLights()                              # standard lighting
        vis.AddGrid(0.5, 0.5, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(2.0, -1.0, 0.0),
                                       chrono.Q_ROTATE_Y_TO_Z),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid (XZ plane)

    # === Main loop === render-cadence outer loop; physics + CSV logging in inner batch
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END    # short check when validating
    os.makedirs("frames", exist_ok=True)                    # guard: review-frame output dir
    os.makedirs("cam", exist_ok=True)                       # guard: mover motion-log dir

    # cache: motor / piston / rod handles fetched once, reused every step
    motor_c = motor
    piston_c = piston
    rod_c = rod

    times, angles, piston_x, piston_vx = [], [], [], []
    data_file = None
    motion_file = None
    try:
        # Two CSVs: simulation_data.csv (key quantities) + cam/motion_log.csv (movers).
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(
            ["time", "crank_angle", "crank_speed", "piston_x", "piston_vx", "rod_angle"]
        )
        motion_writer.writerow(
            ["time", "body", "x", "y", "z", "vx", "vy", "vz"]
        )

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                frame += 1
            for _ in range(RENDER_EVERY):
                t = sys.GetChTime()
                p = piston_c.GetPos()
                v = piston_c.GetPosDt()
                rp = rod_c.GetPos()
                rv = rod_c.GetPosDt()
                ang = motor_c.GetMotorAngle()
                spd = motor_c.GetMotorAngleDt()
                # rod inclination from world X (atan2 of pin-to-pin geometry)
                rod_ang = math.atan2(p.y - rp.y, p.x - rp.x)

                data_writer.writerow([t, ang, spd, p.x, v.x, rod_ang])
                motion_writer.writerow([t, "piston", p.x, p.y, p.z, v.x, v.y, v.z])
                motion_writer.writerow([t, "rod", rp.x, rp.y, rp.z, rv.x, rv.y, rv.z])

                times.append(t)
                angles.append(ang)
                piston_x.append(p.x)
                piston_vx.append(v.x)

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= run_end:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad constraint state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:           # disk / permission while writing CSV
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers so partial output survives an early error.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === plot piston position & velocity vs crank angle
    if times:
        ang_arr = np.array(angles)
        fig, (ax1, ax2) = plt.subplots(2, sharex=True, figsize=(9, 6))
        ax1.plot(ang_arr, np.array(piston_x), color="tab:green")
        ax1.set(ylabel="piston x [m]", title="Crank-slider: piston motion vs crank angle")
        ax1.grid(True)
        ax2.plot(ang_arr, np.array(piston_vx), "r--")
        ax2.set(ylabel="piston vx [m/s]", xlabel="crank angle [rad]")
        ax2.grid(True)
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    print(f"done: {len(times)} steps logged, t_end={sys.GetChTime():.3f}s")


if __name__ == "__main__":
    main()
