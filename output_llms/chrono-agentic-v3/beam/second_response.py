"""
Euler-Bernoulli Beam Simulation with ChBuilderBeamEuler.

Models two beam sections in a PyChrono FEA scene using ChSystemSMC.
A first beam section is built from scratch with manual node/element assembly,
with node 1 fixed via ChLinkMateGeneric constraints and forces applied to
selected nodes. A second Euler-Bernoulli beam is created with ChBuilderBeamEuler
spanning from (0, 0, -0.1) to (0.2, 0, -0.1) with a Y-up direction and 5
elements; its last node is fixed and a downward force (0, -1, 0) is applied to
its first node. Both beams share a common flexible section (wood-like stiffness).
The MKL Pardiso solver and HHT timestepper are used for stiff-beam integration.

System type: ChSystemSMC (required for FEA)
Expected behavior: beams deflect visibly under applied forces and gravity, with
the Euler builder beam bending near its free first node.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl


# === Constants ===
TIME_STEP = 1e-3       # s — stiff Euler beams require 1 ms
SIM_END   = 10.0       # s
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Beam section properties (flexible, wood-like for visible deflection)
SECTION_DIAMETER = 0.02      # m — thin beam for greater flexibility
DENSITY          = 500.0     # kg/m³ — wood-like
YOUNG_MODULUS    = 0.5e9     # Pa — flexible wood-like modulus
POISSON_RATIO    = 0.3

# First beam geometry
BEAM_A = chrono.ChVector3d(0.0,  0.0,  0.0)
BEAM_B = chrono.ChVector3d(0.2,  0.0,  0.0)
NUM_ELEMENTS_FIRST = 5

# Builder beam geometry (Euler-Bernoulli section)
BUILDER_A = chrono.ChVector3d(0.0,  0.0, -0.1)
BUILDER_B = chrono.ChVector3d(0.2,  0.0, -0.1)
NUM_ELEMENTS_BUILDER = 5
BUILDER_UP = chrono.ChVector3d(0, 1, 0)  # Y-up direction for builder beam

# Applied force on builder beam's first node
NODE_FORCE = chrono.ChVector3d(0, -1, 0)

# === System & gravity ===
# FEA scenes use ChSystemSMC with Pardiso MKL solver and HHT timestepper.
# No contact between beams and rigid bodies — no collision system needed.
# FEA beam: no contact material needed — driven by constraints + gravity + applied forces only.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up, standard gravity

# MKL Pardiso solver for stiff Euler-Bernoulli beams
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)

# HHT timestepper — canonical-minimal form for beam elements
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA Mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# === Beam section (shared by both beams) ===
sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(SECTION_DIAMETER)
sec.SetDensity(DENSITY)
sec.SetYoungModulus(YOUNG_MODULUS)
sec.SetShearModulusFromPoisson(POISSON_RATIO)
sec.SetRayleighDamping(0.0)

# === First beam — manual node/element assembly ===
# Build beam from BEAM_A to BEAM_B with NUM_ELEMENTS_FIRST elements.
# Node 1 (hnode1) is fixed via ChLinkMateGeneric instead of SetFixed(True).
beam_length_first = (BEAM_B - BEAM_A).Length()

first_nodes = []
for i in range(NUM_ELEMENTS_FIRST + 1):
    t = i / NUM_ELEMENTS_FIRST
    pos = BEAM_A + (BEAM_B - BEAM_A) * t
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(pos, chrono.QUNIT))
    mesh.AddNode(node)
    first_nodes.append(node)

for i in range(NUM_ELEMENTS_FIRST):
    el = fea.ChElementBeamEuler()
    el.SetNodes(first_nodes[i], first_nodes[i + 1])
    el.SetSection(sec)
    mesh.AddElement(el)

# Fix node 1 (hnode1) via ChLinkMateGeneric — constrain all 6 DOF to a fixed truss.
# (Direct SetFixed(True) is replaced by this constraint-based approach.)
truss1 = chrono.ChBody()
truss1.SetFixed(True)
sys.Add(truss1)

hnode1 = first_nodes[0]  # cache: first node of first beam (the fixed root)
constr1 = chrono.ChLinkMateGeneric()
constr1.Initialize(hnode1, truss1, False, hnode1.Frame(), hnode1.Frame())
constr1.SetConstrainedCoords(True, True, True, True, True, True)  # all 6 DOF
sys.Add(constr1)

# Apply a force to the tip of the first beam (last node)
first_tip = first_nodes[-1]  # cache: tip node of first beam
first_tip.SetForce(chrono.ChVector3d(0, -0.5, 0))

# === Builder beam — Euler-Bernoulli via ChBuilderBeamEuler ===
# Beam from BUILDER_A to BUILDER_B with BUILDER_UP direction and NUM_ELEMENTS_BUILDER elements.
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,
    sec,
    NUM_ELEMENTS_BUILDER,
    BUILDER_A,
    BUILDER_B,
    BUILDER_UP,
)

# Keep a strong reference to the node container to avoid SWIG GC / dangling ptrs.
builder_nodes = builder.GetLastBeamNodes()  # cache: stored before indexing

# Fix the LAST node of the builder beam.
builder_nodes.back().SetFixed(True)

# Apply force (0, -1, 0) to the FIRST node of the builder beam.
builder_nodes.front().SetForce(NODE_FORCE)

# Keep list of builder nodes to avoid SWIG GC during the sim loop.
builder_node_list = [builder_nodes[i] for i in range(builder_nodes.size())]

sys.Add(mesh)

# === FEA Visualization ===
# ChVisualShapeFEA takes mesh as ctor arg; attach before vis.Initialize() for beam pick-up.

# Shape 1 — surface/scalar field: bending moment Mz
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

# === Visualization ===
# Full Irrlicht block: window + Initialize() first, then sky + camera + lights.
# Y-up convention matches the beam FEA world.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Euler-Bernoulli Beam — ChBuilderBeamEuler")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.2, 0.4), chrono.ChVector3d(0.1, 0.0, -0.05))
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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
