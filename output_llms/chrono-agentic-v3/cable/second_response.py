"""
ANCF Cable FEA Simulation — PyChrono 9.0.0, ChSystemSMC, Irrlicht visualization.

Models a flexible ANCF cable beam fixed at one end (back node pinned to a truss),
with a downward force applied at the front node. Uses the MINRES iterative solver
with the EulerImplicitLinearized timestepper. Rayleigh damping is 0.0001.

System: ChSystemSMC (required for FEA)
Solver: ChSolverMINRES (200 iterations, tol=1e-10, diagonal preconditioner, warm start)
Timestepper: ChTimestepperEulerImplicitLinearized
Expected behavior: cable sags under gravity and the applied tip force, oscillating
toward static equilibrium.
"""

# === Imports ===
import os
import math
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP = 0.01         # ANCF cable timestep
SIM_END   = 10.0
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

CABLE_LENGTH   = 0.5
CABLE_DIAMETER = 0.015
CABLE_E        = 0.01e9
CABLE_DAMPING  = 0.0001   # Rayleigh damping (modified from 0.000)
N_ELEMENTS     = 10
TIP_FORCE      = chrono.ChVector3d(0, -0.7, 0)   # modified from -0.2

# === System & gravity ===
# FEA cable demo: Y-up convention, gravity along -Y
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# FEA beam: no SetCollisionSystemType — pure jointed FEA, no rigid-body contact

# === Solver (MINRES) ===
solver = chrono.ChSolverMINRES()
print("Using MINRES solver")
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)
sys.SetSolver(solver)

# === Timestepper (EulerImplicitLinearized for ANCF cable) ===
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# === FEA mesh + ANCF cable section ===
mesh = fea.ChMesh()

sec_cable = fea.ChBeamSectionCable()
sec_cable.SetDiameter(CABLE_DIAMETER)
sec_cable.SetYoungModulus(CABLE_E)
sec_cable.SetRayleighDamping(CABLE_DAMPING)

builder = fea.ChBuilderCableANCF()
builder.BuildBeam(
    mesh, sec_cable, N_ELEMENTS,
    chrono.ChVector3d(0, 0, -0.1),       # A: back end (fixed)
    chrono.ChVector3d(CABLE_LENGTH, 0, -0.1),  # B: front end (loaded)
)

# Keep strong references to beam nodes — SWIG GC pitfall (cache beam nodes)
beam_nodes = builder.GetLastBeamNodes()  # cache: prevents SWIG GC of temporary container
nodes_list = [beam_nodes[i] for i in range(beam_nodes.size())]

# Apply tip force to the front node
front_node = nodes_list[-1]
front_node.SetForce(TIP_FORCE)

sys.Add(mesh)

# === Ground truss and constraint (pin back node) ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Pin back node of cable to truss (ChLinkNodeFrame = 9.0.0 name for ChLinkPointFrame)
hinge = fea.ChLinkNodeFrame()
hinge.Initialize(nodes_list[0], truss)
sys.Add(hinge)

# FEA beam: no contact material needed — driven by constraints + gravity + tip force only

# === FEA Visualization (attach shapes BEFORE vis.Initialize per Irrlicht rules) ===
# Shape 1 — beam bending moment Mz
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node glyphs (coordinate systems)
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — Irrlicht window ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable FEA — MINRES Solver")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up for FEA scenes
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, -0.5, 0.3), chrono.ChVector3d(0.25, 0, -0.1))
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
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
