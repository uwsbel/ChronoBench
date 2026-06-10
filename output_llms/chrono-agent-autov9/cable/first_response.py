"""ANCF cable beam hinged to ground, falling/swinging under gravity (PyChrono 9.0.1).

Model
-----
A flexible cable is modeled with ANCF cable finite elements
(`fea.ChElementCableANCF` built via `fea.ChBuilderCableANCF`). The cable starts
along the +X axis and is pinned at one end ("root") to the fixed world frame;
the rest hangs free. Under gravity the free portion swings down and the cable
deforms into a hanging catenary-like shape, oscillating about the pinned end.

System / solver
---------------
- `ChSystemSMC` (smooth contact system; the recommended host for FEA meshes).
- Direct sparse solver `ChSolverPardisoMKL` (Pardiso/MKL) — required for the
  ill-conditioned FEA stiffness matrices; iterative solvers diverge.
- HHT (Hilber-Hughes-Taylor) implicit timestepper for stable beam integration.

Behavior expected
------------------
The pinned root node stays put; the free tip drops under gravity and swings,
so the tip Z decreases sharply early then oscillates while the root Z holds
constant. Tip and several node positions are logged each step and plotted.

Visualization: Irrlicht window with a deformed-shape FEA visual (node-speed
colormap) plus a node-position glyph overlay.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Constants (geometry / physics) — single source of truth ===
GRAVITY = -9.81             # m/s^2 along -Z (Z-up world)
CABLE_LENGTH = 1.0          # m, total length of the cable
N_ELEMENTS = 16             # number of ANCF cable elements
CABLE_DIAMETER = 0.01       # m, circular cross-section diameter
YOUNG_MODULUS = 1.0e8       # Pa, axial/bending stiffness of the cable
DENSITY = 1000.0            # kg/m^3, cable material density
RAYLEIGH_DAMPING = 0.0001   # internal damping coefficient

TIME_STEP = 5.0e-4          # s, small step for FEA stability
SIM_END = 3.0               # s, total simulated duration
RENDER_FPS = 50.0           # review-frame cadence

ROOT = chrono.ChVector3d(0.0, 0.0, 0.0)            # pinned (hinged) end
TIP_START = chrono.ChVector3d(CABLE_LENGTH, 0.0, 0.0)  # free end, +X

# Derived constants (precomputed once — never recompute in the hot loop)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # steps per frame
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast windowless validation
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check on validate

# Keepalive container: hold strong refs to mesh/builder/section/nodes so the
# SWIG temporaries are not garbage-collected (dangling shared_ptr -> segfault).
KEEPALIVE = []

# === System & gravity === SMC host required for FEA meshes
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, GRAVITY))

# === Solver & timestepper === direct MKL solver + HHT for stable FEA beams
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)
# 9.0.1: select the HHT implicit integrator by type. The base handle returned by
# GetTimestepper() is upcast and exposes no SetAlpha, so we rely on HHT defaults
# (the type switch alone gives stable implicit integration for the cable beam).
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
KEEPALIVE.append(solver)

# === FEA mesh & cable section ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)  # apply gravity loads to FEA nodes

section = fea.ChBeamSectionCable()
section.SetDiameter(CABLE_DIAMETER)
section.SetYoungModulus(YOUNG_MODULUS)
section.SetDensity(DENSITY)
section.SetRayleighDamping(RAYLEIGH_DAMPING)

# === Build the ANCF cable === N elements from ROOT to TIP_START along +X
builder = fea.ChBuilderCableANCF()
builder.BuildBeam(mesh, section, N_ELEMENTS, ROOT, TIP_START)

# FEA cable: no contact material / collision surface needed — the cable is
# driven by gravity + the root hinge constraint only (no rigid-body contact).

# Strong refs to the node container BEFORE indexing (SWIG GC pitfall).
beam_nodes = builder.GetLastBeamNodes()
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]
KEEPALIVE.extend([mesh, section, builder, beam_nodes])
KEEPALIVE.extend(nodes)

root_node = nodes[0]    # pinned end at ROOT
tip_node = nodes[-1]    # free end

sys.Add(mesh)

# === Hinge the root to ground === fix the root node's position to the world.
# A truss body + ChLinkNodeFrame pins the translational DOFs (a hinge) while
# leaving the slope free, so the cable can rotate/swing about the pinned end.
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

hinge = fea.ChLinkNodeFrame()
hinge.Initialize(root_node, truss)
sys.Add(hinge)
KEEPALIVE.extend([truss, hinge])

# === Visualization (FEA visual shapes) === deformed shape + node glyphs
# Deformed mesh colored by node speed magnitude.
vis_mesh = chrono.ChVisualShapeFEA()
vis_mesh.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_mesh.SetColormapRange(chrono.ChVector2d(0.0, 3.0))
vis_mesh.SetSmoothFaces(True)
vis_mesh.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_mesh)

# Node-position glyphs to make individual nodal positions visible.
vis_nodes = chrono.ChVisualShapeFEA()
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.008)
mesh.AddVisualShapeFEA(vis_nodes)
KEEPALIVE.extend([vis_mesh, vis_nodes])

# Indices of nodes sampled into the CSV (root, quarter, mid, three-quarter, tip).
sample_idx = sorted({0, N_ELEMENTS // 4, N_ELEMENTS // 2,
                     (3 * N_ELEMENTS) // 4, N_ELEMENTS})
sample_nodes = [nodes[i] for i in sample_idx]  # cache: node handles fetched once

# === Visualization (Irrlicht window) === full standard scene block
vis = None
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ANCF cable hinged to ground under gravity")
    vis.Initialize()                                    # Initialize FIRST (Irrlicht)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.6, -2.0, 0.3),
                  chrono.ChVector3d(0.5, 0.0, -0.4))     # eye, target
    vis.AddTypicalLights()
    vis.AddGrid(0.2, 0.2, 20, 20,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1.0), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))           # ground reference grid

# === Output setup ===
os.makedirs("frames", exist_ok=True)  # guard against missing frame dir
csv_path = "simulation_data.csv"

# Open with a context manager so the writer always flushes/closes.
try:
    csv_file = open(csv_path, "w", newline="")
except (OSError, IOError) as exc:  # disk full / permission denied
    print(f"Cannot open {csv_path} for writing: {exc}")
    raise

# CSV header: time, root XYZ, tip XYZ, tip speed, then sampled node Z columns.
header = ["time", "root_x", "root_y", "root_z",
          "tip_x", "tip_y", "tip_z", "tip_speed"]
header += [f"node{idx}_z" for idx in sample_idx]

times, tip_z_hist, tip_speed_hist, root_z_hist = [], [], [], []

# === Main loop === render-cadence outer loop, physics in inner batch
frame = 0
try:
    writer = csv.writer(csv_file)
    writer.writerow(header)

    while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
            frame += 1

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            root_p = root_node.GetPos()
            tip_p = tip_node.GetPos()
            tip_v = tip_node.GetPosDt().Length()

            row = [t, root_p.x, root_p.y, root_p.z,
                   tip_p.x, tip_p.y, tip_p.z, tip_v]
            row += [n.GetPos().z for n in sample_nodes]
            writer.writerow(row)

            times.append(t)
            root_z_hist.append(root_p.z)
            tip_z_hist.append(tip_p.z)
            tip_speed_hist.append(tip_v)

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= RUN_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    print(f"Simulation aborted: {exc}")
    raise
finally:
    # Flush + close the CSV writer even if a step diverges mid-run.
    csv_file.flush()
    csv_file.close()

# === Post-processing === time-series plot from the recorded history
try:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.plot(times, tip_z_hist, label="tip Z", color="tab:blue")
    ax1.plot(times, root_z_hist, label="root Z (pinned)", color="tab:red")
    ax1.set_ylabel("height Z [m]")
    ax1.set_title("ANCF cable hinged to ground — falling under gravity")
    ax1.grid(True)
    ax1.legend()

    ax2.plot(times, tip_speed_hist, label="tip speed", color="tab:green")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("tip speed [m/s]")
    ax2.grid(True)
    ax2.legend()

    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)
except (OSError, ValueError) as exc:  # plotting/IO failure must not mask results
    print(f"Plot generation failed: {exc}")

print(f"Done. steps logged={len(times)}  frames written={frame}  "
      f"final tip Z={tip_z_hist[-1] if tip_z_hist else float('nan'):.4f} m")
