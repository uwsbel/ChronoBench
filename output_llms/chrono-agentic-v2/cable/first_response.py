"""
ANCF Cable Beam Simulation — PyChrono 9.0.0 (Irrlicht).

Models a flexible beam composed of ANCF cable elements. One end is hinged
to a fixed ground truss; the beam sags and oscillates under gravity.
The Irrlicht window visualizes beam deformation (bending moment Mz) and
nodal positions (glyph dots) in real time.

System: ChSystemSMC (required for FEA).
Solver: ChSolverSparseQR + ChTimestepperEulerImplicitLinearized (ANCF cable).
"""

# === Imports ===
import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
TIME_STEP   = 0.01        # s  — canonical ANCF cable timestep
SIM_END     = 10.0        # s
RENDER_FPS  = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Cable geometry & material
CABLE_LEN     = 0.5       # m
N_ELEMENTS    = 16        # number of ANCF elements
CABLE_DIA     = 0.015     # m
CABLE_YOUNG   = 0.01e9    # Pa
CABLE_DAMPING = 0.000

# === System & gravity — ChSystemSMC required for FEA (Y-up world) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# FEA beam with no rigid-body contact: no collision system needed.
# FEA beam: no contact material needed — driven by constraints + gravity only.

solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# === FEA mesh — ANCF cable section and builder ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

sec_cable = fea.ChBeamSectionCable()
sec_cable.SetDiameter(CABLE_DIA)
sec_cable.SetYoungModulus(CABLE_YOUNG)
sec_cable.SetRayleighDamping(CABLE_DAMPING)

builder = fea.ChBuilderCableANCF()
builder.BuildBeam(
    mesh, sec_cable, N_ELEMENTS,
    chrono.ChVector3d(0.0,  0.0, 0.0),   # A — hinged end (at origin)
    chrono.ChVector3d(CABLE_LEN, 0.0, 0.0),  # B — free tip
)

# Keep strong reference to node container (SWIG GC pitfall)
beam_nodes_container = builder.GetLastBeamNodes()  # cache: strong ref, prevents GC
nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]

sys.Add(mesh)

# === Constraints — hinge the first node to a fixed ground truss ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

hinge = fea.ChLinkNodeFrame()
hinge.Initialize(nodes[0], truss)
sys.Add(hinge)

# === FEA Visualization — surface (bending moment Mz) + glyph (node dot positions) ===
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — Irrlicht window ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable Beam — Gravity Sag")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.5, -0.3, 0.6),   # eye
    chrono.ChVector3d(0.25, 0.0, 0.0),   # target: beam midpoint
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
