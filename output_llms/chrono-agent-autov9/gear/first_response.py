"""Epicyclic (planetary) gear train simulation in PyChrono 9.0.1 (NSC, Irrlicht).

Model
-----
A planetary gear reducer built from rigid bodies coupled by ideal gear
constraints (``ChLinkLockGear``) rather than meshed tooth collision:

  * truss   : a fixed box that doubles as the stationary internal ring gear (C);
  * bar     : a rotating carrier bar that swings about the truss central axis;
  * gear A  : the sun gear, driven at a CONSTANT angular speed by a rotation-speed
              motor referenced to the fixed truss;
  * gear B  : the planet gear, carried by the bar (revolute to the bar) and meshing
              with both the sun gear A and the fixed ring gear C (the truss).

System type: non-smooth contact (``ChSystemNSC``). The mechanism is purely
constraint-driven (revolutes + gear constraints + one speed motor), so NO contact
collision is enabled.

Expected behavior
-----------------
The motor forces gear A to rotate at a fixed speed. Through the A-B gear mesh and
the B-C epicyclic mesh against the fixed ring, the planet gear B both spins about
its own axis and is dragged around the sun, which makes the carrier bar revolve
about the central axis at a reduced, steady rate. All angular speeds settle to
constant values (a steady planetary kinematic state), which the CSV log and the
time-series plot make visible.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants: timing, geometry, physics (no bare literals downstream) ===
TIME_STEP = 1.0e-3            # integration step [s]
SIM_END = 8.0                # total simulated time [s]
RENDER_FPS = 50.0            # review-frame cadence [frames/s]

MOTOR_SPEED = 6.0            # constant sun-gear angular speed [rad/s]

RAD_A = 2.0                  # sun gear A pitch radius [m]
RAD_B = 4.0                  # planet gear B pitch radius [m]
RAD_C = 2.0 * RAD_B + RAD_A  # fixed internal ring gear C pitch radius [m]
INTERAXIS_AB = RAD_A + RAD_B  # center distance between sun A and planet B [m]

GEAR_THICK_A = 0.5           # axial thickness of gear A [m]
GEAR_THICK_B = 0.4           # axial thickness of gear B [m]
BODY_DENSITY = 1000.0        # rigid-body density [kg/m^3]

TRUSS_SX = 14.0             # truss backing-plate full extents [m]
TRUSS_SY = 14.0
TRUSS_SZ = 0.6
TRUSS_Z = -3.0             # truss center Z: BEHIND the gear plane so it never
                          # occludes the gears (acts as the ring-gear backing plate) [m]

BAR_SX = 8.0                 # carrier bar full extents [m]
BAR_SY = 1.5
BAR_SZ = 1.0

GEAR_PLANE_Z = -1.0          # Z of the sun/planet gear plane [m]
HALF_PI = math.pi / 2.0      # precomputed once: 90 deg in radians

# Derived once: how many physics steps advance between rendered frames.
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Headless fast-validation gate: skip the window and run a short bounded sim.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))

# === System & gravity: one NSC system, gravity along -Z (Z-up world) ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Pure constraint-driven mechanism: no collision/contact, so no collision system.
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Shared contact material (required by the easy-body factories; no contact occurs).
mat = chrono.ChContactMaterialNSC()

# Shared visual material so the gears read clearly in the render.
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))

# === Bodies: fixed truss (ring), rotating carrier bar, sun gear A, planet gear B ===
# Truss: fixed box, also the stationary internal ring gear C of the reducer.
truss = chrono.ChBodyEasyBox(TRUSS_SX, TRUSS_SY, TRUSS_SZ, BODY_DENSITY, True, False, mat)
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0, 0, TRUSS_Z))
sys.Add(truss)

# Carrier bar: rotates about the central axis (revolute to truss at the origin).
bar = chrono.ChBodyEasyBox(BAR_SX, BAR_SY, BAR_SZ, BODY_DENSITY, True, False, mat)
bar.SetPos(chrono.ChVector3d(INTERAXIS_AB / 2.0, 0, 0))
sys.Add(bar)

# Sun gear A: cylinder with Y body-local axis (then tilted so its axis is world Z).
gear_A = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, RAD_A, GEAR_THICK_A, BODY_DENSITY, True, False, mat)
gear_A.SetPos(chrono.ChVector3d(0, 0, GEAR_PLANE_Z))
gear_A.SetRot(chrono.QuatFromAngleX(HALF_PI))
gear_A.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(gear_A)

# Planet gear B: same axial convention, offset by the A-B center distance.
gear_B = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, RAD_B, GEAR_THICK_B, BODY_DENSITY, True, False, mat)
gear_B.SetPos(chrono.ChVector3d(INTERAXIS_AB, 0, GEAR_PLANE_Z))
gear_B.SetRot(chrono.QuatFromAngleX(HALF_PI))
gear_B.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(gear_B)

# === Joints / constraints: carrier revolute, planet revolute, motor, gear meshes ===
# Carrier bar revolves about the truss central axis (hinge axis = world Z at origin).
rev_bar_truss = chrono.ChLinkLockRevolute()
rev_bar_truss.Initialize(truss, bar, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(rev_bar_truss)

# Planet gear B pinned to the carrier bar at the B axis (hinge axis = world Z).
rev_B_bar = chrono.ChLinkLockRevolute()
rev_B_bar.Initialize(gear_B, bar, chrono.ChFramed(chrono.ChVector3d(INTERAXIS_AB, 0, 0), chrono.QUNIT))
sys.AddLink(rev_B_bar)

# Motor: impose a CONSTANT rotation speed on sun gear A relative to the fixed truss.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear_A, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor)

# Gear mesh A<->B. The shaft frames' local +Z is the wheel axis; the easy cylinders
# were built with a Y axis, so rotate each shaft frame -90 deg about X to align.
gear_AB = chrono.ChLinkLockGear()
gear_AB.Initialize(gear_A, gear_B, chrono.ChFramed())
gear_AB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-HALF_PI)))
gear_AB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-HALF_PI)))
gear_AB.SetTransmissionRatio(RAD_A / RAD_B)   # ratio matches the visual cylinder radii
gear_AB.SetEnforcePhase(True)
sys.AddLink(gear_AB)

# Epicyclic mesh B<->C: planet B against the fixed internal ring gear C (the truss).
# Shaft-2 frame sits at the ring center (offset so its axis aligns with the train axis).
gear_BC = chrono.ChLinkLockGear()
gear_BC.Initialize(gear_B, truss, chrono.ChFramed())
gear_BC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-HALF_PI)))
gear_BC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, GEAR_PLANE_Z - TRUSS_Z), chrono.QUNIT))
gear_BC.SetTransmissionRatio(RAD_B / RAD_C)
gear_BC.SetEpicyclic(True)   # internal-teeth ring gear
sys.AddLink(gear_BC)

# === Visualization: full Irrlicht scene (window + sky + camera + lights + grid) ===
# Gated on HEADLESS so the validation run is windowless and fast; the full block
# below is the standard Irrlicht setup (Initialize FIRST, scene elements AFTER).
vis = None
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Epicyclic gear train")
    vis.Initialize()                                    # Initialize FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()                                     # standard sky backdrop
    vis.AddCamera(chrono.ChVector3d(3, -16, 12), chrono.ChVector3d(3, 0, -1))  # AFTER Initialize
    vis.AddTypicalLights()                             # standard lighting
    vis.AddGrid(2.0, 2.0, 30, 30,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TRUSS_Z - TRUSS_SZ), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))         # ground reference grid below the truss

# === Main loop: render-cadence outer loop, physics batch + CSV logging inner ===
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating
os.makedirs("frames", exist_ok=True)                   # guard against missing output dir
os.makedirs("cam", exist_ok=True)                      # motion log target dir

# cache: link handles fetched once, reused every step (avoid repeated lookups)
motor_link = motor
bar_body = bar
gearA_body = gear_A
gearB_body = gear_B

data_csv = None
motion_csv = None
try:
    # Open both CSV writers under context managers so they always flush/close.
    with open("simulation_data.csv", "w", newline="") as data_f, \
         open("cam/motion_log.csv", "w", newline="") as motion_f:
        data_writer = csv.writer(data_f)
        data_writer.writerow([
            "time", "motorA_speed", "barangle", "bar_omega_z",
            "gearA_omega_z", "gearB_omega_z", "constraint_violation",
        ])
        motion_writer = csv.writer(motion_f)
        motion_writer.writerow([
            "time", "body", "x", "y", "z", "wx", "wy", "wz",
        ])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                t = sys.GetChTime()
                bar_w = bar_body.GetAngVelParent()
                ga_w = gearA_body.GetAngVelParent()
                gb_w = gearB_body.GetAngVelParent()
                # Constraint violation: gear-link residual scalar (0 when satisfied).
                try:
                    cviol = gear_AB.GetConstraintViolation().GetNorm()
                except (AttributeError, RuntimeError):
                    cviol = 0.0   # logging-only; never block the run on a debug scalar
                data_writer.writerow([
                    t, motor_link.GetMotorAngleDt(),
                    motor_link.GetMotorAngle(), bar_w.z,
                    ga_w.z, gb_w.z, cviol,
                ])
                bar_p = bar_body.GetPos()
                gb_p = gearB_body.GetPos()
                motion_writer.writerow([t, "bar", bar_p.x, bar_p.y, bar_p.z,
                                        bar_w.x, bar_w.y, bar_w.z])
                motion_writer.writerow([t, "gearB", gb_p.x, gb_p.y, gb_p.z,
                                        gb_w.x, gb_w.y, gb_w.z])

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= run_end:
                    break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:           # disk / permission failure on CSV I/O
    import traceback
    traceback.print_exc()
    raise
finally:
    # The `with` block already flushed/closed both writers on normal or error exit;
    # nothing left open here. (Explicit finally documents the cleanup intent.)
    pass

# === Post-processing: time-series plot of the logged angular speeds ===
try:
    rows = np.genfromtxt("simulation_data.csv", delimiter=",", names=True)
    if rows.size > 0:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax1.plot(rows["time"], rows["gearA_omega_z"], label="gear A omega_z")
        ax1.plot(rows["time"], rows["gearB_omega_z"], label="gear B omega_z")
        ax1.plot(rows["time"], rows["bar_omega_z"], label="carrier bar omega_z")
        ax1.set_ylabel("angular speed [rad/s]")
        ax1.legend(loc="best")
        ax1.grid(True)
        ax2.plot(rows["time"], rows["barangle"], label="motor/sun angle")
        ax2.set_xlabel("time [s]")
        ax2.set_ylabel("angle [rad]")
        ax2.legend(loc="best")
        ax2.grid(True)
        fig.suptitle("Epicyclic gear train kinematics")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)
except (OSError, ValueError) as exc:   # missing CSV / malformed numeric data
    import traceback
    traceback.print_exc()

print("Done. Simulated %.3f s." % sys.GetChTime())
