"""
Multi-chain ANCF cable simulation with connected end bodies.

System type: ChSystemSMC with sparse QR solver and Euler implicit linearized timestepper.
Structure: Model1 class managing n_chains=6 ANCF cable chains, each with:
  - A fixed truss as reference frame
  - A cable beam (n_elems increases per chain)
  - Hinge constraints at the beam start
  - A box body at the beam end connected via ChLinkNodeFrame
  - A second hinge linking beam endpoint to the box

Expected behavior: cables droop under gravity; end boxes swing/settle;
PrintBodyPositions prints end-body positions each step.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP = 0.01         # ANCF cable timestep
SIM_END = 4.0
RENDER_FPS = 50.0
N_CHAINS = 6
CABLE_DIAMETER = 0.015
CABLE_YOUNG = 0.01e9
CABLE_DENSITY = 7800.0
BEAM_LENGTH = 0.5
BOX_MASS = 0.1
BOX_SIZE = 0.02
X_SPACING = 0.15        # spacing between chains along X
Z_OFFSET = 0.0          # base Z level for chain attachments

render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up for FEA

# === Solver: sparse QR + Euler implicit linearized (required for ANCF cable) ===
solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)


class Model1:
    """
    Manages n_chains ANCF cable chains with end boxes.
    Each chain: fixed truss → hinge → cable beam → ChLinkNodeFrame → box body.
    """

    def __init__(self, system, n_chains=6):
        self.system = system
        self.n_chains = n_chains
        self.end_bodies = []         # strong refs to end boxes
        self.meshes = []             # strong refs to meshes (prevent GC)
        self.builders = []           # strong refs to builders (prevent GC)
        self.sections = []           # strong refs to sections
        self.trusses = []            # strong refs to trusses
        self.hinges = []             # strong refs to hinge constraints
        self.end_links = []          # strong refs to end-node body links
        self._build()

    def _build(self):
        for i in range(self.n_chains):
            n_elems = 4 + i          # increasing element count per chain
            x_start = i * X_SPACING

            # --- fixed truss (reference frame) ---
            truss = chrono.ChBody()
            truss.SetFixed(True)
            truss.SetPos(chrono.ChVector3d(x_start, 0, Z_OFFSET))
            self.system.Add(truss)
            self.trusses.append(truss)

            # --- ANCF cable section ---
            sec = fea.ChBeamSectionCable()
            sec.SetDiameter(CABLE_DIAMETER)
            sec.SetYoungModulus(CABLE_YOUNG)
            sec.SetDensity(CABLE_DENSITY)
            sec.SetRayleighDamping(0.0)
            self.sections.append(sec)

            # --- FEA mesh for this chain ---
            mesh = fea.ChMesh()
            mesh.SetAutomaticGravity(True)

            # Build the cable: starts at truss anchor, hangs down
            A = chrono.ChVector3d(x_start, 0.0, Z_OFFSET)
            B = chrono.ChVector3d(x_start, -BEAM_LENGTH, Z_OFFSET)

            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(mesh, sec, n_elems, A, B)

            # Keep strong ref to beam nodes before indexing (SWIG GC pitfall)
            beam_nodes = builder.GetLastBeamNodes()
            all_nodes = [beam_nodes[k] for k in range(beam_nodes.size())]

            # Hinge the start node to the truss
            hinge = fea.ChLinkNodeFrame()
            hinge.Initialize(all_nodes[0], truss)
            self.system.Add(hinge)
            self.hinges.append(hinge)

            # Box body at the end of this chain
            box = chrono.ChBody()
            box.SetMass(BOX_MASS)
            box.SetPos(chrono.ChVector3d(x_start, -BEAM_LENGTH - BOX_SIZE, Z_OFFSET))
            box.SetInertiaXX(chrono.ChVector3d(
                BOX_MASS * BOX_SIZE * BOX_SIZE / 6.0,
                BOX_MASS * BOX_SIZE * BOX_SIZE / 6.0,
                BOX_MASS * BOX_SIZE * BOX_SIZE / 6.0,
            ))
            box_shape = chrono.ChVisualShapeBox(BOX_SIZE, BOX_SIZE, BOX_SIZE)
            box.AddVisualShape(box_shape)
            self.system.Add(box)
            self.end_bodies.append(box)

            # Link end node to box
            end_link = fea.ChLinkNodeFrame()
            end_link.Initialize(all_nodes[-1], box)
            self.system.Add(end_link)
            self.end_links.append(end_link)

            self.system.Add(mesh)
            self.meshes.append(mesh)
            self.builders.append(builder)

            # --- FEA visualization for this mesh ---
            vis_surf = chrono.ChVisualShapeFEA(mesh)
            vis_surf.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
            vis_surf.SetColorscaleMinMax(-0.4, 0.4)
            vis_surf.SetSmoothFaces(True)
            vis_surf.SetWireframe(False)
            mesh.AddVisualShapeFEA(vis_surf)

            vis_glyph = chrono.ChVisualShapeFEA(mesh)
            vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
            vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            vis_glyph.SetSymbolsThickness(0.006)
            vis_glyph.SetSymbolsScale(0.01)
            vis_glyph.SetZbufferHide(False)
            mesh.AddVisualShapeFEA(vis_glyph)

    def PrintBodyPositions(self):
        """Print the position of each chain's end box to stdout."""
        for i, body in enumerate(self.end_bodies):
            p = body.GetPos()
            print(f"  Chain {i}: end body pos = ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")


# === Build the multi-chain model ===
model = Model1(sys, n_chains=N_CHAINS)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Multi-Chain ANCF Cable Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.4, -0.6, 1.2),
    chrono.ChVector3d(0.4, -0.3, 0.0),
)
vis.AddTypicalLights()

# === Review-only setup ===

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            model.PrintBodyPositions()
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
