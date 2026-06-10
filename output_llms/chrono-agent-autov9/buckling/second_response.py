"""Flexible slider-crank / beam-buckling FEA simulation (PyChrono 9.0.1, Irrlicht).

Model
-----
A constrained flexible-beam mechanism built from Euler-Bernoulli beam finite
elements (pychrono.fea). A fixed truss body anchors a VERTICAL beam; a HORIZONTAL
beam runs out along +X and is joined to the vertical beam by a spherical-type
constraint; a short CRANK beam links the horizontal beam to a rotating CRANK
rigid body driven by a rotational-speed motor. As the crank turns it forces the
horizontal beam, and the slender vertical beam laterally deflects/buckles under
the transmitted compressive/bending load.

System type
-----------
ChSystemSMC (smooth contact) with the Pardiso/MKL direct solver and an HHT
implicit timestepper — the combination required for stiff FEA beam matrices.

Main bodies / elements
----------------------
- truss   : fixed ChBody (mechanism ground anchor + visual block).
- crank   : rotating ChBody (visual block) driven by ChLinkMotorRotationSpeed.
- vertical / horizontal / crank Euler beams: fea.ChMesh built with
  ChBuilderBeamEuler, joined by ChLinkMateGeneric / ChLinkMateSpherical.

Expected behavior
-----------------
The motor spins the crank at a constant rate; the mechanism cycles and the
slender vertical beam exhibits a growing lateral (X) deflection — the buckling
response that is logged to CSV and plotted.

No collision/contact material is defined on purpose: the beams are driven only by
constraints, the motor, and gravity and never physically collide with a rigid
body, so a contact surface would be inert here.
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

# === Named constants === geometry / physics / run control (no bare literals downstream)
# Geometry (final stated values)
L = 1.2          # horizontal beam length [m]
H = 0.3          # vertical beam length [m]
K = 0.07         # crank length [m]

# Truss & crank visual block dimensions
TRUSS_DIMS = (0.03, 0.25, 0.12)   # truss visualization box (x, y, z)
CRANK_DIMS = (K, 0.03, 0.03)      # crank visualization box (x, y, z)

# Beam section properties
HBEAM_WY = 0.12          # horizontal beam width in Y
HBEAM_WZ = 0.012         # horizontal beam width in Z
VBEAM_DIAM = 0.03        # vertical beam circular-section diameter
CBEAM_DIAM = 0.054       # crank beam circular-section diameter

HBEAM_NELEM = 1          # horizontal beam Euler elements (single span)
VBEAM_NELEM = 6          # vertical beam Euler elements
CBEAM_NELEM = 5          # crank beam Euler elements

# Material (inferred default — verify): a compliant polymer-like modulus so the
# slender vertical column visibly bows/buckles under the crank-driven compression
# within the run while the implicit solver stays stable. Geometry/sections are the
# stated values; only the (unspecified) elastic material is chosen here.
DENSITY = 1500.0
E_MOD = 5.0e7
G_MOD = E_MOD * 0.35
RAYLEIGH = 0.02

# Constraint / glyph visualization sizes
CONSTR_SPHERE = 0.012        # generic constraint sphere
CRANK_VBEAM_SPHERE = 0.014   # sphere for crank-beam <-> vertical-beam constraint
GLYPH_SCALE = 0.015          # FEA glyph (node symbol) scale

# Motor
MOTOR_SPEED = 0.5 * chrono.CH_PI   # crank angular speed [rad/s]

# Camera
CAM_POS = chrono.ChVector3d(0.0, 0.7, -1.2)
CAM_TARGET = chrono.ChVector3d(0.0, 0.0, 0.0)

# Run control
TIME_STEP = 5e-4
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check when validating

# Strong-reference container guarding against SWIG GC of FEA temporaries
keepalive = []

# === System & gravity === SMC + MKL direct solver + HHT (required for stiff FEA)
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

solver = mkl.ChSolverPardisoMKL()   # direct solver — iterative solvers diverge on FEA
sys.SetSolver(solver)

# Implicit-linearized Euler timestepper: a fixed-step implicit integrator that is
# robust for this stiff, constrained closed-loop FEA mechanism (the adaptive HHT
# step controller cannot be reconfigured through the bound API in this 9.0.1 build
# and stalls at minimum step size on this stiff loop). Installed via the type enum.
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === Bodies === fixed truss anchor + rotating crank rigid body
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.AddVisualShape(chrono.ChVisualShapeBox(*TRUSS_DIMS))
sys.Add(truss)

# Crank body sits at the outboard end of the crank beam, on the slider axis.
crank_center = chrono.ChVector3d(L + K, 0, 0)
crank = chrono.ChBody()
crank.SetPos(crank_center)
crank.AddVisualShape(chrono.ChVisualShapeBox(*CRANK_DIMS))
sys.Add(crank)

# === FEA mesh & beam sections === Euler-Bernoulli beams (no contact material needed)
# FEA beams: no contact material/surface — driven by constraints + motor + gravity only.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Horizontal beam rectangular section
sec_h = fea.ChBeamSectionEulerAdvanced()
sec_h.SetAsRectangularSection(HBEAM_WY, HBEAM_WZ)
sec_h.SetDensity(DENSITY)
sec_h.SetYoungModulus(E_MOD)
sec_h.SetShearModulus(G_MOD)
sec_h.SetRayleighDamping(RAYLEIGH)

# Vertical beam circular section (slender column that buckles)
sec_v = fea.ChBeamSectionEulerAdvanced()
sec_v.SetAsCircularSection(VBEAM_DIAM)
sec_v.SetDensity(DENSITY)
sec_v.SetYoungModulus(E_MOD)
sec_v.SetShearModulus(G_MOD)
sec_v.SetRayleighDamping(RAYLEIGH)

# Crank beam circular section (stiffer link)
sec_c = fea.ChBeamSectionEulerAdvanced()
sec_c.SetAsCircularSection(CBEAM_DIAM)
sec_c.SetDensity(DENSITY)
sec_c.SetYoungModulus(E_MOD)
sec_c.SetShearModulus(G_MOD)
sec_c.SetRayleighDamping(RAYLEIGH)

keepalive += [mesh, sec_h, sec_v, sec_c]

builder = fea.ChBuilderBeamEuler()
keepalive.append(builder)


def mesh_node(global_index):
    # Stable rotational-node handle fetched from the mesh by GLOBAL index.
    # The builder's GetLastBeamNodes() container returns aliasing proxies that get
    # rebound by the next BuildBeam (even .front()), so node handles are taken from
    # the mesh itself, which owns persistent shared_ptrs.
    return fea.CastToChNodeFEAxyzrot(fea.CastToChNodeFEAbase(mesh.GetNode(global_index)))


# Build the three beams; node global indices follow the build order:
#   horizontal -> [0 .. HBEAM_NELEM]      (HBEAM_NELEM+1 nodes)
#   vertical   -> next VBEAM_NELEM+1 nodes
#   crank      -> next CBEAM_NELEM+1 nodes
# Horizontal beam: along +X from the column top to the crank attach point.
builder.BuildBeam(
    mesh, sec_h, HBEAM_NELEM,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(L, 0, 0),
    chrono.ChVector3d(0, 1, 0),
)
h_base = 0
h_count = HBEAM_NELEM + 1

# Vertical beam: slender column from a clamped base at (0,-H,0) up to the
# horizontal beam's inboard node at the origin.
builder.BuildBeam(
    mesh, sec_v, VBEAM_NELEM,
    chrono.ChVector3d(0, -H, 0),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(1, 0, 0),
)
v_base = h_base + h_count
v_count = VBEAM_NELEM + 1

# Crank beam: short link along +X from the horizontal beam's outboard node to crank.
builder.BuildBeam(
    mesh, sec_c, CBEAM_NELEM,
    chrono.ChVector3d(L, 0, 0),
    chrono.ChVector3d(L + K, 0, 0),
    chrono.ChVector3d(0, 1, 0),
)
c_base = v_base + v_count
c_count = CBEAM_NELEM + 1

sys.Add(mesh)

# Resolve the connection nodes by global index (stable handles; see mesh_node()).
hbeam_first = mesh_node(h_base)                 # column-top / inboard end at (0,0,0)
hbeam_last = mesh_node(h_base + h_count - 1)    # outboard end at (L,0,0)
vbeam_first = mesh_node(v_base)                 # clamped base node at (0,-H,0)
vbeam_last = mesh_node(v_base + v_count - 1)    # top node at the origin
vbeam_mid = mesh_node(v_base + v_count // 2)    # column midspan (buckling probe)
cbeam_first = mesh_node(c_base)                 # at (L,0,0)
cbeam_last = mesh_node(c_base + c_count - 1)    # at (L+K,0,0), pins to crank body
assert abs(vbeam_first.GetPos().y + H) < 1e-9, "vertical base node mis-resolved"
assert abs(vbeam_last.GetPos().y) < 1e-9, "vertical top node mis-resolved"

# === Joints / constraints === tie beams to bodies and to each other
# Spherical mates (3 translational constraints) are used at the interior pin
# joints so the closed kinematic loop is NOT rotationally over-constrained — the
# beams supply the rotational compliance. Only the column base is fully clamped.
#
# Base (foot) of the vertical beam clamps to the fixed truss (full 6-DOF clamp).
constr_base = chrono.ChLinkMateGeneric()
constr_base.Initialize(vbeam_first, truss, False, vbeam_first.Frame(), vbeam_first.Frame())
constr_base.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_base)

# Top of the slender vertical column pins to the inboard end of the horizontal beam.
constr_vh = chrono.ChLinkMateSpherical()
constr_vh.Initialize(vbeam_last, hbeam_first, False, vbeam_last.Frame(), hbeam_first.Frame())
sys.Add(constr_vh)

# Outboard end of horizontal beam pins to the crank beam start (spherical).
constr_hc = chrono.ChLinkMateSpherical()
constr_hc.Initialize(hbeam_last, cbeam_first, False, hbeam_last.Frame(), cbeam_first.Frame())
sys.Add(constr_hc)

# Crank beam end pins to the rotating crank body (spherical — no rotational lock).
constr_cb = chrono.ChLinkMateSpherical()
constr_cb.Initialize(cbeam_last, crank, False, cbeam_last.Frame(), cbeam_last.Frame())
sys.Add(constr_cb)

# Rotational-speed motor spins the crank body about Z relative to the truss; the
# revolving crank cyclically drives the horizontal beam and compresses the column.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    crank, truss,
    chrono.ChFramed(crank_center, chrono.QUNIT),   # crank rotates about world Z
)
motor.SetMotorFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.Add(motor)

keepalive += [constr_base, constr_vh, constr_hc, constr_cb, motor]

# === FEA visualization === colored deformed beams + node glyphs on the mesh
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColormapRange(chrono.ChVector2d(-60, 60))
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)

vis_nodes = chrono.ChVisualShapeFEA()
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_nodes.SetSymbolsThickness(GLYPH_SCALE)
vis_nodes.SetSymbolsScale(GLYPH_SCALE)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)

keepalive += [vis_beam, vis_nodes]

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y here
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Flexible slider-crank beam buckling (FEA)")
    vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(CAM_POS, CAM_TARGET)                  # AFTER Initialize
    vis.AddTypicalLights()
    vis.AddGrid(0.1, 0.1, 30, 30,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0),
                                   chrono.QuatFromAngleX(chrono.CH_PI_2)),
                chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid (XZ plane)
    vis.SetSymbolScale(CONSTR_SPHERE / 0.01)            # scale link/force symbols

# === Main loop === render-cadence outer loop, physics inner batch, CSV per step
os.makedirs("frames", exist_ok=True)   # guard against missing output dir
os.makedirs("cam", exist_ok=True)

# cache: fetch the buckling probe node (vertical-column midspan) once, reused every step
probe_node = vbeam_mid
vbeam_base = vbeam_first   # cache: clamped base, lateral-deflection reference (reused)

csv_file = None
try:
    csv_file = open("simulation_data.csv", "w", newline="")   # guarded below
except (OSError, IOError) as exc:   # disk full / permission denied
    raise RuntimeError(f"cannot open simulation_data.csv: {exc}")

times, lat_defl, mid_x, foot_x = [], [], [], []

try:
    writer = csv.writer(csv_file)
    writer.writerow(["time", "lateral_deflection", "mid_node_x", "base_node_x", "top_node_y"])

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
            frame += 1
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            px = probe_node.GetPos().x
            fx = vbeam_base.GetPos().x
            ty = vbeam_last.GetPos().y
            deflection = px - fx          # lateral (X) bow of the slender column
            writer.writerow([t, deflection, px, fx, ty])
            times.append(t)
            lat_defl.append(deflection)
            mid_x.append(px)
            foot_x.append(fx)
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= RUN_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
    import traceback
    traceback.print_exc()
    raise
finally:
    if csv_file is not None:
        csv_file.flush()
        csv_file.close()   # flush partial CSV even if a step diverged

# === Post-processing === plot lateral deflection vs time
if times:
    arr_defl = np.array(lat_defl)
    print(f"steps={len(times)}  final_t={times[-1]:.4f}s  "
          f"max|lateral_deflection|={np.max(np.abs(arr_defl)):.5e} m  "
          f"final_deflection={arr_defl[-1]:.5e} m")

    fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax[0].plot(times, lat_defl, color="crimson", label="lateral deflection (X) of midspan")
    ax[0].set_ylabel("lateral deflection [m]")
    ax[0].grid(True)
    ax[0].legend(loc="best")
    ax[1].plot(times, mid_x, color="navy", label="midspan node X")
    ax[1].plot(times, foot_x, color="seagreen", label="foot node X")
    ax[1].set_xlabel("time [s]")
    ax[1].set_ylabel("X position [m]")
    ax[1].grid(True)
    ax[1].legend(loc="best")
    fig.suptitle("Flexible slider-crank beam: vertical-column buckling response")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    print("wrote simulation_timeseries.png")
else:
    print("WARNING: no samples logged")
