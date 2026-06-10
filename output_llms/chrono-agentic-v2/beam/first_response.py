"""
Beam FEA Simulation — PyChrono 9.0.x with Irrlicht

Models a cantilever beam using Euler-Bernoulli finite elements. A ChMesh is
populated with Euler beam elements and nodes using ChBuilderBeamEuler. The
root node is fixed; a force and torque are applied to the free tip node. The
system uses ChSystemSMC with Pardiso MKL solver and HHT timestepper (canonical
for stiff beam elements). Two ChVisualShapeFEA shapes render the bending-moment
field (Mz) and node coordinate-system glyphs. The Irrlicht window shows the
deforming beam in real time.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
time_step = 1e-3          # 1 ms — canonical for stiff Euler beams
sim_end   = 10.0          # seconds
render_fps   = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Beam geometry and material
BEAM_LENGTH  = 0.4        # m
BEAM_DIAM    = 0.012      # m (circular cross-section)
BEAM_DENSITY = 7800.0     # kg/m³ (steel)
BEAM_E       = 210e9      # Pa (Young's modulus — steel)
BEAM_NU      = 0.3        # Poisson ratio
BEAM_DAMP    = 0.000      # Rayleigh damping
N_ELEMENTS   = 16         # number of Euler elements along the beam

TIP_FORCE    = chrono.ChVector3d(0,  4,  2)   # N — lateral tip load
TIP_TORQUE   = chrono.ChVector3d(0,  0, -0.04)# Nm — tip torque

# === System — SMC required for all FEA truths ===
# FEA beam: no contact material needed — driven by constraints + gravity + motor only
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up gravity

# Pardiso MKL direct solver — required for stiff Euler/IGA beams
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper — canonical-minimal form (matches SimBench truth)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Euler-Bernoulli beam section
sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(BEAM_DIAM)
sec.SetDensity(BEAM_DENSITY)
sec.SetYoungModulus(BEAM_E)
sec.SetShearModulusFromPoisson(BEAM_NU)
sec.SetRayleighDamping(BEAM_DAMP)

# Build beam: horizontal along X axis, root at origin, tip at (BEAM_LENGTH, 0, 0)
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh, sec, N_ELEMENTS,
    chrono.ChVector3d(0.0, 0.0, 0.0),           # root
    chrono.ChVector3d(BEAM_LENGTH, 0.0, 0.0),    # tip
    chrono.ChVector3d(0, 1, 0),                  # section Y direction (up)
)

# Keep strong reference to node container — SWIG GC pitfall: must not index temp
beam_nodes = builder.GetLastBeamNodes()

# Fix the root node (cantilever boundary condition)
beam_nodes.front().SetFixed(True)

# Apply tip load
tip_node = beam_nodes.back()
tip_node.SetForce(TIP_FORCE)
tip_node.SetTorque(TIP_TORQUE)

sys.Add(mesh)

# === FEA Visualization — TWO shapes: bending-moment surface + node-csys glyphs ===
# Shape 1: bending moment Mz coloured surface
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

# === Visualization — Irrlicht window ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono FEA — Euler Beam")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                     # FIRST — inverse of VSG
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.2, 0.6),
              chrono.ChVector3d(0.2, 0.0, 0.0))      # AFTER Initialize
vis.AddTypicalLights()

# === Review-only setup ===

# === Main simulation loop ===
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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
