"""Jeffcott rotor modeled with an Isogeometric Analysis (IGA) Cosserat beam.

Model
-----
A slender, flexible shaft is discretized with IGA (NURBS) beam elements
(`ChBuilderBeamIGA` + `ChBeamSectionCosseratEasyCircular`). A heavy rigid
flywheel is rigidly attached to the shaft mid-span; the flywheel carries a small
mass eccentricity so that, once the shaft spins, the rotating unbalance excites
the classic Jeffcott whirl (the shaft center traces a circular/elliptical orbit
in the plane normal to the spin axis). One end of the shaft is driven by a
constant-speed rotational motor (`ChLinkMotorRotationSpeed`); the far end is
pinned in translation to a fixed support so the shaft can bow but cannot fly off.

System type
-----------
`ChSystemSMC` (required for FEA) with the MKL/Pardiso direct solver and a
linearized implicit-Euler timestepper (both appropriate for the stiff beam
stiffness matrix of a continuously driven rotor).

Main bodies
-----------
- IGA Cosserat beam shaft (FEA mesh, `ChNodeFEAxyzrot` nodes).
- Rigid flywheel `ChBody` at mid-span (eccentric COG).
- Two fixed truss bodies acting as the drive bearing and the support bearing.

Expected behavior
------------------
The motor brings the shaft up to the target spin speed about the shaft (X) axis.
The eccentric flywheel deflects the flexible shaft laterally; the mid-span node
whirls, producing a near-sinusoidal Y/Z displacement orbit logged to CSV. Output:
per-step CSV of the mid-span node displacement and whirl radius, a matplotlib
time-series PNG, and (interactive run) Irrlicht review frames of the spinning
beam + flywheel with FEM color visualization.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Constants (geometry / physics) =========================================
# Named constants only — no bare positional literals downstream.
SHAFT_LENGTH = 1.0           # m, total shaft span along +X
N_ELEMENTS = 12              # IGA beam elements along the shaft
IGA_ORDER = 3                # NURBS order (cubic)
SHAFT_DIAMETER = 0.012       # m, slender flexible shaft
SHAFT_E = 2.0e11             # Pa, steel Young's modulus
SHAFT_G = 7.9e10             # Pa, steel shear modulus
SHAFT_DENSITY = 7800.0       # kg/m^3, steel
SHAFT_DAMPING = 0.01         # Rayleigh damping coefficient

FLYWHEEL_MASS = 3.0          # kg, heavy disk at mid-span
FLYWHEEL_RADIUS = 0.08       # m, disk radius (visual + inertia)
FLYWHEEL_THICKNESS = 0.02    # m, disk thickness
FLYWHEEL_ECC = 2.0e-3        # m, COG eccentricity -> rotating unbalance (whirl)

SPIN_SPEED = 60.0            # rad/s, commanded shaft spin (~573 rpm)
GRAVITY = -9.81              # m/s^2 along Z

TIME_STEP = 5.0e-4           # s, small step for FEA stability
SIM_END = 4.0                # s, total simulated time
RENDER_FPS = 50.0            # review frame cadence

# Derived geometry (precomputed once)
X_START = 0.0
X_END = SHAFT_LENGTH
X_MID = 0.5 * SHAFT_LENGTH
START_POS = chrono.ChVector3d(X_START, 0.0, 0.0)
END_POS = chrono.ChVector3d(X_END, 0.0, 0.0)
MID_POS = chrono.ChVector3d(X_MID, 0.0, 0.0)
Y_DIR = chrono.ChVector3d(0, 1, 0)   # IGA section lateral reference direction

# Headless validation gate: a fast, windowless physics check.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short check when validating
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# Keep strong Python references against SWIG garbage collection (CRITICAL for FEA).
KEEPALIVE = []

# === System & gravity ========================================================
# ChSystemSMC + MKL direct solver: required for stiff FEA beam matrices.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY))
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)
KEEPALIVE.append(solver)
# Linearized implicit-Euler timestepper: robust for the stiff, continuously
# rotating beam loop. (An adaptive HHT integrator stalls at its minimum step on
# this driven-rotation problem; the linearized implicit Euler steps stably at a
# fixed small TIME_STEP, the appropriate choice for a motor-forced rotor.)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA mesh: IGA Cosserat beam shaft =======================================
# FEA beam: no contact material needed — driven by constraints + motor + gravity only.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
KEEPALIVE.append(mesh)

section = fea.ChBeamSectionCosseratEasyCircular(
    SHAFT_DIAMETER, SHAFT_E, SHAFT_G, SHAFT_DENSITY
)
# Structural Rayleigh damping for the Cosserat section (beta on the stiffness).
section_damping = fea.ChDampingCosseratRayleigh(section.GetElasticity(), SHAFT_DAMPING)
section.SetDamping(section_damping)
KEEPALIVE.append(section_damping)
KEEPALIVE.append(section)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, section, N_ELEMENTS, START_POS, END_POS, Y_DIR, IGA_ORDER)
KEEPALIVE.append(builder)

# Resolve node handles immediately and keep strong refs (SWIG GC pitfall):
# the GetLastBeamNodes() container is a temporary; copy out before it is freed.
beam_nodes_container = builder.GetLastBeamNodes()
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]
KEEPALIVE.extend(beam_nodes)
node_drive = beam_nodes[0]          # motor-driven end node
node_support = beam_nodes[-1]       # pinned support end node

# Pick the node closest to mid-span as the whirl probe (its X position is fixed).
mid_index = min(
    range(len(beam_nodes)),
    key=lambda i: abs(beam_nodes[i].GetPos().x - X_MID),
)
node_mid = beam_nodes[mid_index]
sys.Add(mesh)

# === Bodies: flywheel + fixed truss bearings =================================
# Drive-end truss (fixed) carries the rotational motor.
truss_drive = chrono.ChBody()
truss_drive.SetFixed(True)
truss_drive.SetPos(START_POS)
sys.Add(truss_drive)
KEEPALIVE.append(truss_drive)

# Support-end truss (fixed) provides the far bearing reaction.
truss_support = chrono.ChBody()
truss_support.SetFixed(True)
truss_support.SetPos(END_POS)
sys.Add(truss_support)
KEEPALIVE.append(truss_support)

# Rigid flywheel disk near mid-span. Its mass center is offset from the shaft
# axis by FLYWHEEL_ECC, so when the shaft spins the disk acts as a rotating
# unbalance -> Jeffcott whirl. (This build's ChBody has no COM-frame setter, so
# the eccentricity is realized by placing the whole disk body off-axis.)
FLYWHEEL_POS = chrono.ChVector3d(X_MID, FLYWHEEL_ECC, 0.0)
flywheel = chrono.ChBody()
flywheel.SetPos(FLYWHEEL_POS)
flywheel.SetMass(FLYWHEEL_MASS)
# Disk inertia about its own axes (X = spin axis along the shaft).
Ixx = 0.5 * FLYWHEEL_MASS * FLYWHEEL_RADIUS ** 2
Iyy = 0.25 * FLYWHEEL_MASS * FLYWHEEL_RADIUS ** 2 + \
    (1.0 / 12.0) * FLYWHEEL_MASS * FLYWHEEL_THICKNESS ** 2
flywheel.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Iyy))
disk_shape = chrono.ChVisualShapeCylinder(FLYWHEEL_RADIUS, FLYWHEEL_THICKNESS)
disk_shape.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
# Cylinder default axis is Z; rotate so the disk axis aligns with the shaft (X).
rot_y2x = chrono.QuatFromAngleY(chrono.CH_PI_2)
flywheel.AddVisualShape(disk_shape, chrono.ChFramed(chrono.VNULL, rot_y2x))
sys.Add(flywheel)
KEEPALIVE.append(flywheel)

# === Joints / constraints / motor ============================================
# Rigidly fix the flywheel to the mid-span beam node (all 6 DOF). The shared link
# frame sits on the shaft axis at the node, so the off-axis disk spins with it.
link_flywheel = chrono.ChLinkMateGeneric()
link_flywheel.Initialize(flywheel, node_mid, False,
                         chrono.ChFramed(MID_POS, chrono.QUNIT),
                         chrono.ChFramed(MID_POS, chrono.QUNIT))
link_flywheel.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(link_flywheel)
KEEPALIVE.append(link_flywheel)

# Far end: pin translation to the fixed support (shaft may tilt, not translate).
link_support = chrono.ChLinkMateGeneric()
link_support.Initialize(truss_support, node_support, False,
                        chrono.ChFramed(END_POS, chrono.QUNIT),
                        chrono.ChFramed(END_POS, chrono.QUNIT))
link_support.SetConstrainedCoords(True, True, True, False, False, False)
sys.Add(link_support)
KEEPALIVE.append(link_support)

# Constant-speed rotational motor drives the drive-end node about the shaft (X).
# Motor frame: rotate Z->X so the motor's spin axis points along the shaft.
motor = chrono.ChLinkMotorRotationSpeed()
motor_frame = chrono.ChFramed(START_POS, chrono.QuatFromAngleY(chrono.CH_PI_2))
motor.Initialize(node_drive, truss_drive, motor_frame)
motor.SetSpeedFunction(chrono.ChFunctionConst(SPIN_SPEED))
sys.Add(motor)
KEEPALIVE.append(motor)

# === Visualization (FEM color + flywheel) — full Irrlicht scene ==============
# FEM beam visualization: speed-norm colormap + undeformed wireframe overlay.
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_beam.SetColormapRange(chrono.ChVector2d(0.0, SPIN_SPEED * FLYWHEEL_RADIUS))
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)
KEEPALIVE.append(vis_beam)

vis_wire = chrono.ChVisualShapeFEA()
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_wire.SetWireframe(True)
vis_wire.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(vis_wire)
KEEPALIVE.append(vis_wire)

vis = None
if not HEADLESS:
    # Full Irrlicht block: window + Initialize() + logo + sky + camera + lights + grid.
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # gravity along -Z
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Jeffcott Rotor — IGA Beam + Flywheel")
    vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(X_MID, -1.0, 0.6),
                  chrono.ChVector3d(X_MID, 0.0, 0.0))   # AFTER Initialize
    vis.AddTypicalLights()
    vis.AddGrid(0.1, 0.1, 30, 30,
                chrono.ChCoordsysd(chrono.ChVector3d(X_MID, 0, -0.4), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

# === Main loop (render-cadence outer, physics inner) =========================
os.makedirs("frames", exist_ok=True)   # guard against missing output dir

# cache: fetch the mid-span node handle once; reused every step (no re-lookup).
probe = node_mid
mid_x = X_MID                           # precomputed once: probe's fixed X coord

csv_file = None
try:
    try:
        csv_file = open("simulation_data.csv", "w", newline="")  # disk / permission guard
    except (OSError, IOError) as exc:    # cannot open output CSV
        print(f"Cannot open simulation_data.csv: {exc}")
        raise
    writer = csv.writer(csv_file)
    writer.writerow(["time", "mid_y", "mid_z", "whirl_radius",
                     "motor_angle", "motor_speed"])

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
            pos = probe.GetPos()                 # mid-span node lateral deflection
            whirl = math.hypot(pos.y, pos.z)     # whirl orbit radius in Y-Z plane
            writer.writerow([t, pos.y, pos.z, whirl,
                             motor.GetMotorAngle(), motor.GetMotorAngleDt()])
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= RUN_END:
                break
except (RuntimeError, ValueError) as exc:    # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    if csv_file is not None:
        csv_file.close()                 # flush partial CSV even on mid-run failure

# === Post-processing: time-series plot =======================================
try:
    data = np.genfromtxt("simulation_data.csv", delimiter=",", names=True)
    if data.size > 0:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax1.plot(data["time"], data["mid_y"] * 1e3, label="mid Y (mm)")
        ax1.plot(data["time"], data["mid_z"] * 1e3, label="mid Z (mm)")
        ax1.plot(data["time"], data["whirl_radius"] * 1e3,
                 "k--", label="whirl radius (mm)")
        ax1.set_ylabel("deflection (mm)")
        ax1.legend(loc="upper right")
        ax1.grid(True)
        ax1.set_title("Jeffcott rotor mid-span whirl (IGA beam)")
        ax2.plot(data["time"], data["motor_speed"], "g-", label="motor speed (rad/s)")
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("spin speed (rad/s)")
        ax2.legend(loc="lower right")
        ax2.grid(True)
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)
except (OSError, IOError, ValueError) as exc:   # plotting / file read failure
    print(f"Post-processing plot skipped: {exc}")

print(f"Done. Simulated to t={sys.GetChTime():.3f}s, "
      f"nodes={mesh.GetNumNodes()}, elements={mesh.GetNumElements()}.")
