"""
ANCF Cable Beam FEA Simulation — PyChrono 9.0.0 (ChSystemSMC, Irrlicht)

Models a flexible cable hanging under gravity, built with ANCF cable beam
elements (ChBeamSectionCable + ChBuilderCableANCF). One end of the cable is
pinned to a fixed truss; a downward force is applied to the free (front) node.

Configuration (this file):
  - Rayleigh damping: 0.0001
  - Applied force on front node: (0, -0.7, 0) N
  - Solver: ChSolverMINRES (max 200 iters, tol 1e-10, diagonal preconditioner,
    warm start, verbose OFF)
  - Timestepper: ChTimestepperEulerImplicitLinearized (standard for ANCF cable)
  - System type: ChSystemSMC (required for FEA)
  - World convention: Y-up (gravity along -Y), standard for Chrono FEA demos

Expected behaviour: the cable sags and swings under gravity + the applied
downward nodal force, exhibiting the flexible bending typical of ANCF cable
elements with moderate damping.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# === Simulation parameters ===
TIME_STEP = 0.01          # ANCF cable standard timestep
SIM_END   = 10.0          # simulation duration [s]
RENDER_FPS = 30.0         # review video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Cable geometry
CABLE_LENGTH  = 0.5       # cable total length [m]
CABLE_DIAM    = 0.015     # cable cross-section diameter [m]
CABLE_YOUNG   = 0.01e9   # Young's modulus [Pa]
CABLE_DAMPING = 0.0001    # Rayleigh damping coefficient (changed from 0.000)
N_ELEMENTS    = 10        # number of ANCF elements

# Applied load on front (free) node
FRONT_FORCE = chrono.ChVector3d(0, -0.7, 0)  # changed from (0, -0.2, 0)

# === System & gravity (Y-up; FEA ground truth uses Y-up) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure ANCF cable FEA: no rigid-body contact — SetCollisionSystemType not needed

# === Solver: ChSolverMINRES (replacing ChSolverSparseQR) ===
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)
sys.SetSolver(solver)
print("Using MINRES solver")

# === Timestepper: EulerImplicitLinearized (standard for ANCF cable) ===
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA mesh: ANCF cable ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Cable section properties
sec_cable = fea.ChBeamSectionCable()
sec_cable.SetDiameter(CABLE_DIAM)
sec_cable.SetYoungModulus(CABLE_YOUNG)
sec_cable.SetRayleighDamping(CABLE_DAMPING)

# Build the ANCF cable beam from (0,0,-0.1) to (CABLE_LENGTH, 0, -0.1)
builder = fea.ChBuilderCableANCF()
builder.BuildBeam(
    mesh,
    sec_cable,
    N_ELEMENTS,
    chrono.ChVector3d(0, 0, -0.1),
    chrono.ChVector3d(CABLE_LENGTH, 0, -0.1),
)

# SWIG GC guard: store beam node container before indexing
beam_nodes = builder.GetLastBeamNodes()
all_nodes  = [beam_nodes[i] for i in range(beam_nodes.size())]  # cache: prevent GC

# Apply force on the free (front) node — last node in builder order
front_node = all_nodes[-1]
front_node.SetForce(FRONT_FORCE)

sys.Add(mesh)

# === Fixed truss & pin constraint ===
# Pin the back node (first in builder order) to the fixed world truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

hinge = fea.ChLinkNodeFrame()
hinge.Initialize(all_nodes[0], truss)
sys.Add(hinge)

# FEA cable: no contact surface needed — driven by constraints + gravity + applied force only

# === FEA visualization (two-shape pattern for 9.0.0) ===
# Shape 1 — scalar field: bending moment Mz along the cable
vis_surface = chrono.ChVisualShapeFEA()              # no-arg constructor in 9.0.0
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColormapRange(-0.4, 0.4)              # actual API in 9.0.0
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node glyphs: dot positions at each node
vis_glyph = chrono.ChVisualShapeFEA()               # no-arg constructor in 9.0.0
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization: Irrlicht window (Y-up, cable hangs in XY plane) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable FEA — MINRES Solver, Rayleigh=0.0001, Force=-0.7N")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up for FEA demos
vis.Initialize()                                     # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.0, 0.6),     # AFTER Initialize
              chrono.ChVector3d(0.25, 0.0, -0.1))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -0.3, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
