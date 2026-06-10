"""
Euler-Bernoulli FEA beam simulation with two beam segments.

System type: ChSystemSMC (required for FEA)
Solver: Pardiso MKL (required for stiff beam elements)
Timestepper: HHT (canonical for Euler beam elements)

Two beam segments are built using ChBuilderBeamEuler:
  Segment 1: from (0, 0, 0) to (0.1, 0.1, 0) with Y-up direction (1, 0, 0),
             root node fixed, force/torque applied at the tip node.
  Segment 2: from the last node of segment 1 (the 'A' node, i.e. builder's last
             back node) to endpoint (0.2, 0.1, -0.1) with Y-up direction (0, 1, 0).

The mesh visualizes bending moment Mz (colored surface) and node coordinate
systems (glyph overlay). Expected behavior: the two-segment beam deforms under
the applied tip force and torque.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Constants ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity ===
# FEA beam scenes use ChSystemSMC + Pardiso MKL direct solver
# No contact collision needed: pure jointed FEA, no rigid-body contact
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up world

# Pardiso MKL solver: required for stiff Euler-Bernoulli beam stiffness matrices
mkl_solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(mkl_solver)

# HHT timestepper: canonical-minimal form for stiff beam elements
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA Mesh & beam sections ===
# FEA beam: no contact material needed — driven by constraints + gravity + applied loads only
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Beam cross-section: circular Euler-Bernoulli beam (aluminium-like)
sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(0.01)          # 10 mm diameter
sec.SetDensity(2700.0)                  # aluminium density kg/m³
sec.SetYoungModulus(73e9)               # aluminium Young's modulus Pa
sec.SetShearModulusFromPoisson(0.3)     # derived from Poisson ratio 0.3
sec.SetRayleighDamping(0.000)

# === Segment 1: (0,0,0) -> (0.1, 0.1, 0), up=(1,0,0) ===
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh, sec, 5,
    chrono.ChVector3d(0.0, 0.0, 0.0),    # start point A
    chrono.ChVector3d(0.1, 0.1, 0.0),    # end point B
    chrono.ChVector3d(1.0, 0.0, 0.0),    # 'Y' up direction for section
)

# Keep strong reference to node container (SWIG GC pitfall)
seg1_nodes = builder.GetLastBeamNodes()
seg1_node_list = [seg1_nodes[i] for i in range(seg1_nodes.size())]  # cache: strong refs

# Fix the root node of segment 1
seg1_node_list[0].SetFixed(True)

# Apply force and torque at the tip of segment 1
seg1_tip = seg1_node_list[-1]  # cache: tip node of segment 1
seg1_tip.SetForce(chrono.ChVector3d(4.0, 2.0, 0.0))    # N
seg1_tip.SetTorque(chrono.ChVector3d(0.0, -0.04, 0.0))  # Nm

# === Segment 2: last node of seg1 (A) -> (0.2, 0.1, -0.1), up=(0,1,0) ===
# The 'A' start node is the last node (back) of segment 1
builder.BuildBeam(
    mesh, sec, 5,
    seg1_tip.GetPos(),                    # A: position of seg1's last node
    chrono.ChVector3d(0.2, 0.1, -0.1),   # B: endpoint
    chrono.ChVector3d(0.0, 1.0, 0.0),    # 'Y' up direction as (0, 1, 0)
)

# Keep strong references to segment 2 node container
seg2_nodes = builder.GetLastBeamNodes()
seg2_node_list = [seg2_nodes[i] for i in range(seg2_nodes.size())]  # cache: strong refs

# Constrain the first node of segment 2 to be co-located with last node of segment 1
# Use a fixed truss as ground reference for the constraint
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Fix the junction: weld segment 2's root node to the seg1 tip position via ChLinkMateFix
# This ensures continuity between the two beam segments
weld = chrono.ChLinkMateFix()
weld.Initialize(seg2_node_list[0], seg1_tip)
sys.Add(weld)

sys.Add(mesh)

# === FEA Visualization ===
# Shape 1: surface colored by bending moment Mz
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)  # two scalars (lo, hi)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2: node coordinate system glyphs
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
vis.SetWindowTitle("FEA Euler Beam — Two Segments")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.2, 0.3), chrono.ChVector3d(0.1, 0.05, 0.0))
vis.AddTypicalLights()

# === Review-only recording setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad numeric state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
