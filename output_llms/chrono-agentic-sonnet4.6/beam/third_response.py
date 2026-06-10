"""
FEA Euler-Bernoulli beam simulation with two connected beam segments.

System: ChSystemSMC with Pardiso MKL solver and HHT timestepper.
Beams: Two Euler-Bernoulli beam segments sharing a common node.
  - Beam 1: fixed at origin, extends to (0.1, 0, 0).
  - Beam 2: starts from Beam 1's last node, ends at (0.2, 0.1, -0.1).
Both beams use a circular cross-section with aluminum-like properties.
Expected behavior: Beams deform under gravity; segments droop and oscillate
showing FEA bending behavior. The connecting node couples the two segments.

FEA beam: no contact material needed — driven by constraints + gravity only.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Constants ===
TIME_STEP = 1e-3        # s — stiff beam timestep
SIM_END   = 5.0         # s
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

BEAM_DIAMETER = 0.05    # m
BEAM_DENSITY  = 2700.0  # kg/m³ (aluminium)
BEAM_E        = 73e9    # Pa
BEAM_NU       = 0.3     # Poisson ratio
BEAM_DAMP     = 0.0

N_ELEMENTS_1 = 4        # elements in beam segment 1
N_ELEMENTS_2 = 4        # elements in beam segment 2

# Beam endpoints
A1 = chrono.ChVector3d(0.0, 0.0,  0.0)
B1 = chrono.ChVector3d(0.1, 0.0,  0.0)
# Beam 2: A2 = last node of beam 1 (= B1), B2 given by prompt
B2 = chrono.ChVector3d(0.2, 0.1, -0.1)

UP_DIR = chrono.ChVector3d(0, 1, 0)   # 'Y' up direction for both beams

# === System & gravity ===
# Y-up world for FEA beam demos (gravity along -Y)
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Pardiso MKL solver — required for stiff Euler beam stiffness matrices
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)

# HHT timestepper — canonical minimal form for stiff beams
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA Mesh & section ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Cross-section shared by both beam segments
sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(BEAM_DIAMETER)
sec.SetDensity(BEAM_DENSITY)
sec.SetYoungModulus(BEAM_E)
sec.SetShearModulusFromPoisson(BEAM_NU)
sec.SetRayleighDamping(BEAM_DAMP)

# === Beam segment 1: A1 → B1 ===
builder1 = fea.ChBuilderBeamEuler()
builder1.BuildBeam(
    mesh, sec, N_ELEMENTS_1,
    A1, B1,
    UP_DIR,
)

# Keep strong reference to node container to avoid SWIG GC segfault
beam1_nodes = builder1.GetLastBeamNodes()

# Fix root node of beam 1 (cantilever boundary condition)
root_node = beam1_nodes.front()
root_node.SetFixed(True)

# Last node of beam 1 — used as starting node for beam 2
last_node_beam1 = beam1_nodes.back()

# === Beam segment 2: last node of beam 1 → B2 ===
# Use a second builder; the 'A' node comes from beam 1's last node
builder2 = fea.ChBuilderBeamEuler()
builder2.BuildBeam(
    mesh, sec, N_ELEMENTS_2,
    last_node_beam1.GetPos(),   # start at beam1's end position
    B2,
    UP_DIR,
)

# Keep strong reference to beam 2 node container
beam2_nodes = builder2.GetLastBeamNodes()
beam2_tip = beam2_nodes.back()

# Connect beam 2's first node to beam 1's last node with a ChLinkMateGeneric
# (all 6 DOF constrained — rigid junction between segments)
truss_ground = chrono.ChBody()
truss_ground.SetFixed(True)
sys.Add(truss_ground)

# Weld beam1 last node to beam2 first node via a shared-node approach:
# The builder already built beam2 starting from the spatial position of last_node_beam1,
# but they are separate nodes. We join them with a full 6-DOF generic constraint.
beam2_first_node = beam2_nodes.front()
junction = chrono.ChLinkMateGeneric()
junction.Initialize(
    last_node_beam1, beam2_first_node,
    False,
    last_node_beam1.Frame(),
    last_node_beam1.Frame(),
)
junction.SetConstrainedCoords(True, True, True,   # tx, ty, tz
                              True, True, True)   # rx, ry, rz
sys.Add(junction)

sys.Add(mesh)

# === FEA Visualization ===
# Shape 1: beam bending moment Mz (surface/scalar field)
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2: node coordinate-system glyphs
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA Two-Segment Euler Beam")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.3, 0.3, 0.4), chrono.ChVector3d(0.15, 0.05, -0.05))
vis.AddTypicalLights()

# === Recording setup (review-only) ===


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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad FEA state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
