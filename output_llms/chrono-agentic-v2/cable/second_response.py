"""
ANCF Cable Simulation with MINRES Solver — PyChrono 9.0.0 / ChSystemSMC

Models a flexible cable (ANCF elements) clamped at one end with a transverse
force applied at the free (front) node. Uses ChSolverMINRES with diagonal
preconditioner and warm start for iterative sparse solving.

System: ChSystemSMC (Y-up, gravity disabled on mesh)
Cable section: ChBeamSectionCable with Rayleigh damping 0.0001
Force: 0.7 N downward at the free (front) node
Solver: MINRES, 200 iterations, tol=1e-10
Timestepper: ChTimestepperEulerImplicitLinearized
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Simulation constants ===
TIME_STEP = 0.01          # ANCF cable timestep (s)
SIM_END   = 5.0           # simulation duration (s)
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Cable geometry & material
CABLE_LENGTH   = 0.5        # m
CABLE_DIAMETER = 0.015      # m
CABLE_DENSITY  = 8000.0     # kg/m³ (steel-like)
CABLE_E        = 0.01e9     # Young's modulus (Pa)
CABLE_DAMPING  = 0.0001     # Rayleigh damping (modified from 0.000)
N_ELEMENTS     = 10

# Applied force at front node
FORCE_FRONT = chrono.ChVector3d(0, -0.7, 0)  # N (modified from -0.2)

# === System & gravity (Y-up, FEA gravity disabled on mesh) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Solver: MINRES (modified from ChSolverSparseQR) ===
solver = chrono.ChSolverMINRES()
print("Using MINRES solver")
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)
sys.SetSolver(solver)

# === Timestepper: Euler implicit linearized (required for ANCF cable) ===
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# === FEA Mesh: ANCF cable ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)  # disable gravity on FEA mesh; use applied force instead

# Cable section
sec_cable = fea.ChBeamSectionCable()
sec_cable.SetDiameter(CABLE_DIAMETER)
sec_cable.SetYoungModulus(CABLE_E)
sec_cable.SetDensity(CABLE_DENSITY)
sec_cable.SetRayleighDamping(CABLE_DAMPING)

# Build ANCF cable beam
builder = fea.ChBuilderCableANCF()
builder.BuildBeam(
    mesh,
    sec_cable,
    N_ELEMENTS,
    chrono.ChVector3d(0, 0, -0.1),           # start node (back/clamped end)
    chrono.ChVector3d(CABLE_LENGTH, 0, -0.1), # end node (front/free end)
)

# Keep strong references to nodes (SWIG GC pitfall)
beam_nodes_container = builder.GetLastBeamNodes()  # cache: fetched once
all_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]

# Fix the back (last) node to ground via a truss
back_node = all_nodes[-1]  # clamped end
front_node = all_nodes[0]  # free end with applied force

# Apply transverse force at the free (front) node
front_node.SetForce(FORCE_FRONT)

sys.Add(mesh)

# === Ground truss (fixed body) for cable attachment ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
sys.Add(truss)

# Pin back node to truss with ChLinkNodeFrame (translational constraint)
hinge = fea.ChLinkNodeFrame()
hinge.Initialize(back_node, truss)
sys.Add(hinge)

# FEA cable: no contact material needed — no rigid-body contact in this pure cable scene.

# === FEA Visualization (two-shape canonical pattern) ===
# Shape 1: surface/scalar field (beam bending moment Mz)
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2: node glyph (coordinate system triads)
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization: Irrlicht window ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up convention for FEA
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable - MINRES Solver")
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.5, -0.3, 0.4),   # eye
    chrono.ChVector3d(0.25, 0, -0.1),    # look-at target (cable midpoint region)
)
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
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
