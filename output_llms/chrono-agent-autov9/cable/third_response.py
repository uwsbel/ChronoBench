"""ANCF cable chains hanging from a fixed truss, each tethered to a falling box.

Model
-----
This script builds several flexible cables modelled with Chrono's FEA ANCF
cable elements (`fea.ChBuilderCableANCF` + `fea.ChBeamSectionCable`). A small
class, `Model1`, owns the whole assembly: it loops `n_chains` times and, for
each chain, creates a fixed truss reference body, builds an ANCF cable whose
element count grows with the chain index, pins the cable's first node to the
truss with a `fea.ChLinkNodeFrame` hinge, applies a small tip force, and then
connects the cable's last node to a free rigid box (`ChBody`) with a second
`fea.ChLinkNodeFrame`. The boxes therefore hang from the cables and swing under
gravity, deforming the cables.

System type
-----------
`ChSystemSMC` with the PardisoMKL direct solver and the HHT timestepper — the
combination required for stable ANCF/FEA stiffness matrices (iterative solvers
diverge). Gravity acts along -Z (Z-up world).

Main bodies
-----------
- `n_chains` fixed truss bodies (one anchor per chain).
- `n_chains` ANCF cable meshes (deformable).
- `n_chains` free rigid end boxes, each linked to its cable tip.

Expected behavior
-----------------
At t=0 the cables are horizontal; under gravity each cable sags and its end box
falls and swings, the longer (more-element) cables sagging more. `Model1`'s
`PrintBodyPositions` reports each end box position every step. The run logs the
end-box trajectories to `simulation_data.csv` and plots them to
`simulation_timeseries.png`. Smooth, bounded, NaN-free motion is the success
criterion.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib

matplotlib.use("Agg")  # headless-safe backend for the post-run plot
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Named constants ===
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run

TIME_STEP = 5.0e-4          # small step required for ANCF/FEA stability
SIM_END = 4.0               # seconds of simulated motion
RENDER_FPS = 30.0           # review-video frame rate
GRAVITY = chrono.ChVector3d(0, 0, -9.81)

N_CHAINS = 6                # number of independent cable chains (Model1 default)
BASE_ELEMENTS = 4           # element count of the shortest (first) chain
CABLE_LENGTH = 1.0          # horizontal span of every cable (m)
CHAIN_SPACING = 0.6         # Y offset between successive chains (avoid overlap)

CABLE_DIAMETER = 0.015      # cable cross-section diameter (m)
CABLE_DENSITY = 1000.0      # kg/m^3
CABLE_YOUNG = 0.01e9        # Young's modulus (Pa) — compliant cable
CABLE_RAYLEIGH = 0.000      # structural damping coefficient

BOX_SIZE = 0.10             # end-box edge length (m)
BOX_MASS = 0.10             # end-box mass (kg)
TIP_FORCE = chrono.ChVector3d(0, 0, -0.07)  # small downward tip load (N)

TRUSS_X = 0.0               # anchor X for every chain
TRUSS_Z = 0.0               # anchor Z (cables start horizontal at z=0)

# Derived constants (precomputed once — never recomputed in the loop)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short check when validating


# === Model assembly (Model1) ===
class Model1:
    """Builds and owns the multi-chain ANCF cable assembly inside `sys`.

    `n_chains` controls how many cable chains are generated. Strong references
    to every Chrono object are retained on the instance so the SWIG wrapper does
    not garbage-collect the underlying shared_ptrs (a known FEA segfault source).
    """

    def __init__(self, sys, n_chains=N_CHAINS):
        self.sys = sys
        self.n_chains = n_chains

        # Keepalive containers — prevent premature SWIG GC of FEA objects.
        self.meshes = []
        self.sections = []
        self.builders = []
        self.trusses = []
        self.end_boxes = []     # the rigid end body of each chain
        self.end_nodes = []     # the ANCF tip node of each chain
        self.links = []

        for i in range(self.n_chains):
            self._build_chain(i)

    def _build_chain(self, i):
        """Create one truss + ANCF cable + end box and constrain them together."""
        y = i * CHAIN_SPACING  # lateral position of this chain (no overlap)

        # --- Fixed truss: per-chain anchor reference frame ---
        truss = chrono.ChBody()
        truss.SetFixed(True)
        truss.SetPos(chrono.ChVector3d(TRUSS_X, y, TRUSS_Z))
        truss_box = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
        truss_box.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
        truss.AddVisualShape(truss_box)
        self.sys.Add(truss)
        self.trusses.append(truss)

        # --- ANCF cable section (no contact material: free cable, gravity + links only) ---
        section = fea.ChBeamSectionCable()
        section.SetDiameter(CABLE_DIAMETER)
        section.SetYoungModulus(CABLE_YOUNG)
        section.SetDensity(CABLE_DENSITY)
        section.SetRayleighDamping(CABLE_RAYLEIGH)
        self.sections.append(section)

        # --- ANCF cable mesh; element count grows with chain index ---
        mesh = fea.ChMesh()
        mesh.SetAutomaticGravity(True)
        n_elements = BASE_ELEMENTS + i  # increasing resolution per chain

        start = chrono.ChVector3d(TRUSS_X, y, TRUSS_Z)
        end = chrono.ChVector3d(TRUSS_X + CABLE_LENGTH, y, TRUSS_Z)

        builder = fea.ChBuilderCableANCF()
        builder.BuildBeam(mesh, section, n_elements, start, end)
        self.builders.append(builder)

        # Store the node container BEFORE indexing (SWIG GC pitfall).
        beam_nodes = builder.GetLastBeamNodes()
        node_first = beam_nodes.front()
        node_last = beam_nodes.back()
        self.end_nodes.append(node_last)

        self.sys.Add(mesh)
        self.meshes.append(mesh)
        self._add_mesh_visuals(mesh)

        # --- Boundary condition: hinge the first node to the fixed truss ---
        hinge = fea.ChLinkNodeFrame()
        hinge.Initialize(node_first, truss)
        self.sys.Add(hinge)
        self.links.append(hinge)

        # --- Load: small downward tip force on the last cable node ---
        node_last.SetForce(TIP_FORCE)

        # --- End body: a free rigid box constrained to the cable tip ---
        box = chrono.ChBody()
        box.SetMass(BOX_MASS)
        box.SetPos(end)
        box_shape = chrono.ChVisualShapeBox(BOX_SIZE, BOX_SIZE, BOX_SIZE)
        box_shape.SetColor(chrono.ChColor(0.2, 0.5 + 0.07 * i, 0.8))
        box.AddVisualShape(box_shape)
        self.sys.Add(box)
        self.end_boxes.append(box)

        # Constraint between the beam endpoint and the box.
        tip_link = fea.ChLinkNodeFrame()
        tip_link.Initialize(node_last, box)
        self.sys.Add(tip_link)
        self.links.append(tip_link)

    def _add_mesh_visuals(self, mesh):
        """Attach FEA visual shapes (speed colormap + node glyphs) to a cable mesh."""
        vis_speed = chrono.ChVisualShapeFEA()
        vis_speed.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
        vis_speed.SetColormapRange(chrono.ChVector2d(0.0, 1.5))
        vis_speed.SetSmoothFaces(True)
        mesh.AddVisualShapeFEA(vis_speed)

        vis_nodes = chrono.ChVisualShapeFEA()
        vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        vis_nodes.SetSymbolsThickness(0.008)
        mesh.AddVisualShapeFEA(vis_nodes)

    def PrintBodyPositions(self):
        """Print the position of each chain's end body at the current step."""
        t = self.sys.GetChTime()
        parts = [f"t={t:7.4f}"]
        for i, box in enumerate(self.end_boxes):
            p = box.GetPos()  # cache: fetched once per box per call
            parts.append(f"chain{i}=({p.x:6.3f},{p.y:6.3f},{p.z:6.3f})")
        print(" | ".join(parts))

    def end_positions(self):
        """Return a flat list [x0,y0,z0, x1,y1,z1, ...] of end-box positions."""
        flat = []
        for box in self.end_boxes:
            p = box.GetPos()  # cache: single getter call reused below
            flat.extend([p.x, p.y, p.z])
        return flat


# === System & gravity ===
# ANCF/FEA needs an SMC system with a direct (MKL) solver and HHT integration.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(GRAVITY)
sys.SetSolver(mkl.ChSolverPardisoMKL())  # direct solver required for FEA stiffness
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)  # 9.0.1: HHT via type setter

# === Bodies / cables / constraints ===
model = Model1(sys, n_chains=N_CHAINS)  # cache: built once, reused every step

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)  # Z-up world; before Initialize
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ANCF cable chains with end bodies")
    vis.Initialize()  # Initialize FIRST, then add scene elements (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    cam_eye = chrono.ChVector3d(CABLE_LENGTH * 0.5, -3.2,
                                (N_CHAINS - 1) * CHAIN_SPACING * 0.5 + 0.5)
    cam_target = chrono.ChVector3d(CABLE_LENGTH * 0.5, 0.0,
                                   -(N_CHAINS - 1) * CHAIN_SPACING * 0.25)
    vis.AddCamera(cam_eye, cam_target)  # AFTER Initialize
    vis.AddTypicalLights()
    vis.AddGrid(0.25, 0.25, 24, 24,
                chrono.ChCoordsysd(chrono.ChVector3d(CABLE_LENGTH * 0.5,
                                                     (N_CHAINS - 1) * CHAIN_SPACING * 0.5,
                                                     -2.0),
                                   chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))  # ground reference grid

# === Main loop ===
os.makedirs("frames", exist_ok=True)  # guard against missing output dir

csv_header = ["time"]
for i in range(N_CHAINS):
    csv_header += [f"box{i}_x", f"box{i}_y", f"box{i}_z"]

csv_file = None
try:
    csv_file = open("simulation_data.csv", "w", newline="")  # context-managed below
except (OSError, IOError) as exc:  # disk full / permission denied
    print(f"Could not open CSV for writing: {exc}")
    raise

times = []
trajectories = []  # list of flat end-position rows

try:
    with csv_file:  # ensures the writer is flushed and closed on any exit
        writer = csv.writer(csv_file)
        writer.writerow(csv_header)

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
                row = model.end_positions()
                writer.writerow([t] + row)
                times.append(t)
                trajectories.append(row)
                model.PrintBodyPositions()  # required: report end bodies each step
                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= RUN_END:
                    break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid FEA state
    import traceback
    traceback.print_exc()
    print(f"Simulation aborted: {exc}")
    raise
finally:
    # CSV already closed by the `with` block above; nothing else to flush here.
    print(f"Logged {len(times)} steps; final sim time {sys.GetChTime():.4f} s")

# === Post-processing ===
# Plot each end box's vertical position vs time so the sag/swing is visible.
if times:
    traj = np.asarray(trajectories)
    t_arr = np.asarray(times)
    fig, ax = plt.subplots(figsize=(9, 5))
    for i in range(N_CHAINS):
        ax.plot(t_arr, traj[:, 3 * i + 2], label=f"chain {i} box z")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("end-box height z (m)")
    ax.set_title("ANCF cable chains — end-body vertical motion")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=120)
    plt.close(fig)
    print("Wrote simulation_timeseries.png")
