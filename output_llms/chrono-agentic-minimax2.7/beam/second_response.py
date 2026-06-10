"""
Euler-Bernoulli Beam FEA Simulation
===================================
Plan: FEA beam spanning from (0,0,-0.1) to (0.2,0,-0.1) with Y-up direction,
5 elements, last node fixed via ChLinkMateGeneric, force (0,-1,0) applied to first node.
System: ChSystemSMC with Pardiso MKL solver.
"""

import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# ---------------------------------------------------------------------------
# Geometry & physics constants
# ---------------------------------------------------------------------------
beam_start = chrono.ChVector3d(0, 0, -0.1)
beam_end = chrono.ChVector3d(0.2, 0, -0.1)
beam_up = chrono.ChVector3d(0, 1, 0)          # Y-up section direction
n_elements = 5
time_step = 1e-3
sim_end = 5.0

# ---------------------------------------------------------------------------
# System creation — FEA uses ChSystemSMC + Pardiso MKL
# ---------------------------------------------------------------------------
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Direct Pardiso MKL solver (stiff beam stiffness matrices require direct solve)
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper for stiff beam elements
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# ---------------------------------------------------------------------------
# FEA mesh and Euler beam section
# ---------------------------------------------------------------------------
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Euler-Bernoulli beam section — circular aluminium beam
sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(diameter=0.03)
sec.SetDensity(2700.0)                     # aluminium
sec.SetYoungModulus(73e9)
sec.SetShearModulusFromPoisson(0.3)
sec.SetRayleighDamping(0.0)

# Build the beam using the builder helper
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh, sec, n_elements,
    beam_start,
    beam_end,
    beam_up,
)

# Store beam nodes to keep strong references (SWIG GC safeguard)
beam_nodes = builder.GetLastBeamNodes()
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]

# Fix the LAST node via ChLinkMateFix
truss = chrono.ChBody()
truss.SetFixed(True)
sys.AddBody(truss)

last_node = beam_nodes.back()
weld = chrono.ChLinkMateFix()
weld.Initialize(last_node, truss)
sys.Add(weld)

# Apply force (0, -1, 0) to the FIRST node
first_node = beam_nodes.front()
first_node.SetForce(chrono.ChVector3d(0, -1, 0))

sys.Add(mesh)

# ---------------------------------------------------------------------------
# FEA visualisation shapes — MUST be added BEFORE vis.Initialize()
# ---------------------------------------------------------------------------
vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-0.4, 0.4)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# ---------------------------------------------------------------------------
# Visualization — full Irrlicht block
# ---------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Euler-Bernoulli Beam FEA")
vis.Initialize()

vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -0.5, 1.0), chrono.ChVector3d(0.1, 0, -0.1))
vis.AddTypicalLights()

# ---------------------------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------------------------
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

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

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
