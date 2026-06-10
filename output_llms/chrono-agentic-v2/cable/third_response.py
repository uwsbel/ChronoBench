"""
ANCF Cable Chains Simulation — Multiple Chains with Connected Bodies (ChSystemSMC).

Models 6 chains of ANCF cable beam elements. Each chain:
  - Has a fixed truss as reference frame
  - Is built with an increasing number of cable elements per chain
  - Is pinned at its start node via a hinge constraint (ChLinkNodeFrame) to the truss
  - Has a box body attached to its endpoint via a hinge constraint
  - Has a vertical downward force applied to its endpoint node
Gravity is Y-up: (0, -9.81, 0). Solver: sparse QR + Euler implicit linearized.
Expected behavior: chains sag under gravity; endpoint boxes sway; positions printed each step.
"""

# === Imports ===
import os
import math
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
TIME_STEP  = 0.01         # ANCF cable timestep (seconds)
SIM_END    = 5.0          # simulation duration (seconds)
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Chain layout parameters
N_CHAINS       = 6         # number of cable chains
CHAIN_LENGTH   = 0.5       # cable length (m)
BASE_ELEMENTS  = 4         # elements in chain 0; chain i gets BASE_ELEMENTS + i
CHAIN_SPACING  = 0.7       # horizontal spacing between chains (m)
Y_START        = 0.0       # Y position of the fixed pin point
BOX_SIDE       = 0.04      # side length of endpoint box (m)
BOX_MASS       = 0.1       # kg
CABLE_DIAMETER = 0.015
CABLE_E        = 0.01e9    # Pa
CABLE_DAMPING  = 0.000
END_FORCE_Y    = -3.0      # N, downward applied force on endpoint node

# === System setup (ChSystemSMC for FEA) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Sparse QR solver + Euler implicit linearized for ANCF cable
solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# FEA mesh (all chains share one mesh for efficiency)
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
# FEA cable: no contact material needed — chains driven by gravity + hinge constraints only

# === Model1 class — builds N_CHAINS cable chains with endpoint boxes ===

class Model1:
    """Builds and owns all chains; tracks endpoint bodies for position printing."""

    def __init__(self, n_chains=N_CHAINS):
        self.n_chains     = n_chains
        self.end_bodies   = []    # strong references to endpoint boxes (prevent GC)
        self.meshes_ref   = []    # keep mesh reference alive
        self.trusses      = []    # fixed reference bodies
        self.builders     = []    # ChBuilderCableANCF instances
        self.sections     = []    # ChBeamSectionCable instances
        self.hinges_start = []    # hinge at cable start (node -> truss)
        self.hinges_end   = []    # hinge at cable end (node -> box)
        self._build(n_chains)

    def _build(self, n_chains):
        """Build all cable chains and attach them to the global sys/mesh."""
        for i in range(n_chains):
            x_offset = i * CHAIN_SPACING
            n_elems  = BASE_ELEMENTS + i   # increasing element count per chain

            # Fixed truss (reference frame / ground point for this chain)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetPos(chrono.ChVector3d(x_offset, Y_START, 0))
            sys.Add(mtruss)
            self.trusses.append(mtruss)

            # Cable section (reuse parameters for all chains)
            sec = fea.ChBeamSectionCable()
            sec.SetDiameter(CABLE_DIAMETER)
            sec.SetYoungModulus(CABLE_E)
            sec.SetRayleighDamping(CABLE_DAMPING)
            self.sections.append(sec)

            # Build cable: horizontal, displaced in X by x_offset, hanging in Y
            A = chrono.ChVector3d(x_offset, Y_START, 0)
            B = chrono.ChVector3d(x_offset + CHAIN_LENGTH, Y_START, 0)
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(mesh, sec, n_elems, A, B)
            self.builders.append(builder)

            # CRITICAL: keep reference to GetLastBeamNodes() before indexing (SWIG GC)
            beam_nodes = builder.GetLastBeamNodes()
            node_start = beam_nodes[0]
            node_end   = beam_nodes[beam_nodes.size() - 1]

            # Apply downward force at the endpoint node
            node_end.SetForce(chrono.ChVector3d(0, END_FORCE_Y, 0))

            # Hinge: pin start node to the fixed truss
            hinge_s = fea.ChLinkNodeFrame()
            hinge_s.Initialize(node_start, mtruss)
            sys.Add(hinge_s)
            self.hinges_start.append(hinge_s)

            # Endpoint box body
            box = chrono.ChBodyEasyBox(BOX_SIDE, BOX_SIDE, BOX_SIDE, BOX_MASS / (BOX_SIDE**3))
            box.SetPos(node_end.GetPos() + chrono.ChVector3d(BOX_SIDE * 0.5, 0, 0))
            sys.Add(box)
            self.end_bodies.append(box)

            # Hinge: pin end node to box body
            hinge_e = fea.ChLinkNodeFrame()
            hinge_e.Initialize(node_end, box)
            sys.Add(hinge_e)
            self.hinges_end.append(hinge_e)

        sys.Add(mesh)
        self.meshes_ref.append(mesh)

    def PrintBodyPositions(self):
        """Print the positions of the endpoint box bodies for each chain."""
        for i, body in enumerate(self.end_bodies):
            p = body.GetPos()
            print(f"  Chain {i}: end body pos = ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")


# === Build the model ===
model = Model1(n_chains=N_CHAINS)

# === FEA Visualization ===
# Surface/scalar shape: beam bending moment Mz
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Glyph shape: node coordinate systems
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — full Irrlicht scene ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up scene
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable Chains — Multiple Chains with Connected Bodies")
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.0, -2.0, 2.0), chrono.ChVector3d(2.0, -0.5, 0))
vis.AddTypicalLights()

# === Review-only setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
        model.PrintBodyPositions()
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
