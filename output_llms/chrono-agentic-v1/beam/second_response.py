"""
Euler-Bernoulli Beam FEA Simulation with ChBuilderBeamEuler and ChLinkMateGeneric.

System type: ChSystemSMC (required for stiff FEA Euler-Bernoulli beams).
Models two Euler-Bernoulli beam sections built with ChBuilderBeamEuler:
  1. Beam A — runs from (0,0,0) to (0.2,0,0) with 4 elements; its root (first) node
     is fully constrained (all 6 DOF) using ChLinkMateGeneric against a fixed truss,
     replacing a direct SetFixed() call as specified.
  2. Beam B — runs from (0,0,-0.1) to (0.2,0,-0.1) with 5 elements and Y as the
     section-up direction; its last node is fixed via SetFixed(True) and a
     concentrated force (0,-1,0) N is applied at its first node.
Solver: MKL Pardiso direct solver (required for stiff beam stiffness matrices).
Timestepper: HHT (canonical-minimal form: SetStepControl(False) only).
FEA contact: none needed — beams driven by constraints and nodal force only.
Expected behaviour: Beam A bends under gravity (root fixed via ChLinkMateGeneric);
Beam B deforms under the applied -Y point force at its free (first) end.
"""

import os
import csv

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


# === Constants & parameters ===
TIME_STEP  = 1e-3    # s — stiff Euler beam canonical timestep
SIM_END    = 3.0     # s
RENDER_FPS = 30.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Beam A geometry (ChLinkMateGeneric root constraint)
BEAM_A_START = chrono.ChVector3d(0.0,  0.0, 0.0)
BEAM_A_END   = chrono.ChVector3d(0.2,  0.0, 0.0)
BEAM_A_NELEMS = 4

# Beam B geometry (ChBuilderBeamEuler with SetFixed on last node + point force on first)
BEAM_B_START = chrono.ChVector3d(0.0,  0.0, -0.1)
BEAM_B_END   = chrono.ChVector3d(0.2,  0.0, -0.1)
BEAM_B_NELEMS = 5
BEAM_B_FORCE  = chrono.ChVector3d(0.0, -1.0, 0.0)   # N at first node

# Section material — aluminium-like thin rod
SEC_DIAMETER = 0.012   # m  circular cross-section
SEC_DENSITY  = 2700.0  # kg/m³
SEC_YOUNG    = 73e9    # Pa
SEC_POISSON  = 0.3
SEC_RAYLEIGH = 0.0

# === System & gravity ===
# ChSystemSMC is the correct system for FEA beams; Y-up convention matches FEA demos.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
# FEA beam: no contact material and no collision system needed — pure constraints + forces.

# === Solver: MKL Pardiso (direct, required for stiff beam stiffness matrices) ===
sys.SetSolver(mkl.ChSolverPardisoMKL())

# === Timestepper: HHT canonical-minimal form ===
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# --- Shared Euler-Bernoulli section ---
sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(SEC_DIAMETER)
sec.SetDensity(SEC_DENSITY)
sec.SetYoungModulus(SEC_YOUNG)
sec.SetShearModulusFromPoisson(SEC_POISSON)
sec.SetRayleighDamping(SEC_RAYLEIGH)

# ---------------------------------------------------------------
# === Beam A — ChBuilderBeamEuler; root fixed via ChLinkMateGeneric ===
# Spans from BEAM_A_START to BEAM_A_END with BEAM_A_NELEMS elements.
# The first (root) node is fully constrained via ChLinkMateGeneric, replacing
# a direct hnode.SetFixed(True) call as required.
# ---------------------------------------------------------------
builder_a = fea.ChBuilderBeamEuler()
builder_a.BuildBeam(
    mesh, sec, BEAM_A_NELEMS,
    BEAM_A_START,
    BEAM_A_END,
    chrono.VECT_Y,  # section-up direction
)

# Keep strong references (SWIG GC pitfall: temporary container becomes dangling).
beam_a_nodes = builder_a.GetLastBeamNodes()                           # cache: node container
beam_a_node_list = [beam_a_nodes[i] for i in range(beam_a_nodes.size())]

# Fixed-body truss to anchor the ChLinkMateGeneric constraint (beam A root).
truss_a = chrono.ChBody()
truss_a.SetFixed(True)
sys.Add(truss_a)

# Fully constrain root node of Beam A using ChLinkMateGeneric (6 DOF locked).
# This replaces the approach of calling root_node.SetFixed(True) directly.
root_node_a = beam_a_node_list[0]   # cache: root node of beam A
constr_a = chrono.ChLinkMateGeneric()
constr_a.Initialize(root_node_a, truss_a, False,
                    root_node_a.Frame(), root_node_a.Frame())
constr_a.SetConstrainedCoords(True, True, True,   # constrain tx, ty, tz
                              True, True, True)   # constrain rx, ry, rz
sys.Add(constr_a)

# ---------------------------------------------------------------
# === Beam B — ChBuilderBeamEuler from (0,0,-0.1) to (0.2,0,-0.1), 5 elements ===
# Section-up direction: Y (as specified).
# Last node: SetFixed(True).
# First node: concentrated force (0,-1,0) N applied.
# ---------------------------------------------------------------
builder_b = fea.ChBuilderBeamEuler()
builder_b.BuildBeam(
    mesh, sec, BEAM_B_NELEMS,
    BEAM_B_START,
    BEAM_B_END,
    chrono.VECT_Y,   # section-up direction: Y
)

# Keep strong references (SWIG GC pitfall).
beam_b_nodes = builder_b.GetLastBeamNodes()                           # cache: node container
beam_b_node_list = [beam_b_nodes[i] for i in range(beam_b_nodes.size())]

# Fix the LAST node of Beam B.
last_node_b = beam_b_node_list[-1]   # cache: last node
last_node_b.SetFixed(True)

# Apply concentrated force to the FIRST node of Beam B.
first_node_b = beam_b_node_list[0]   # cache: first node
first_node_b.SetForce(BEAM_B_FORCE)

# Register mesh with the system.
sys.Add(mesh)

# ---------------------------------------------------------------
# === FEA visualization (attached to mesh BEFORE vis.Initialize()) ===
# Shape 1: beam bending moment Mz — colored field
# Shape 2: node coordinate-system glyphs
# ---------------------------------------------------------------
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# ---------------------------------------------------------------
# === Visualization — Irrlicht window ===
# Order: configure → Initialize() → scene elements (Irrlicht inverse order).
# FEA demos are Y-up; use CameraVerticalDir_Y.
# ---------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Euler-Bernoulli Beam FEA — ChBuilderBeamEuler + ChLinkMateGeneric")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.1, 0.2, 0.4),    # eye: view both beams
    chrono.ChVector3d(0.1, 0.0, -0.05),  # target: between the two beams
)
vis.AddTypicalLights()
vis.AddGrid(
    0.05, 0.05, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, -0.06, 0.0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# ---------------------------------------------------------------
# === Review-only setup: frame capture and CSV logging ===
# ---------------------------------------------------------------


# ---------------------------------------------------------------
# === Main simulation loop ===
# Outer: render once per frame; Inner: advance physics render_every steps.
# ---------------------------------------------------------------
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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad integration state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
