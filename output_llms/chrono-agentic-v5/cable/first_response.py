"""ANCF cable beam hinged to ground, deforming under gravity (PyChrono, Irrlicht).

Models a flexible cable as a chain of ANCF cable elements (fea.ChBeamSectionCable +
fea.ChBuilderCableANCF). One end node is hinged to a fixed ground/truss body via a
ChLinkNodeFrame (3 translational DOF), and the whole mesh is subjected to gravity, so
the free end swings down and the cable sags / deforms. The deformed shape (bending
moment colour field) and the nodal positions (coordinate-system glyphs) are visualized
inside the real-time Irrlicht loop.

System type: ChSystemSMC (FEA standard). World convention: Y-up, gravity (0, -9.81, 0).
Solver/timestepper for ANCF cable: ChSolverSparseQR + ChTimestepperEulerImplicitLinearized.
No contact material / collision: the cable is driven only by the hinge constraint and
gravity, so it never collides with a rigid body.
Expected behavior: the cable, pinned at one end, falls and oscillates under gravity,
showing a curved deformed profile.
"""

import os
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# === Parameters === geometry / material / time constants (no bare literals downstream)
time_step = 0.01            # ANCF cable stable step (skill-recommended)
sim_end = 5.0               # seconds of simulated motion
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))          # precomputed once

cable_length = 1.0          # m, total cable span
n_elements = 10             # number of ANCF cable elements
cable_diameter = 0.015      # m, circular cross-section
cable_E = 0.01e9            # Pa, low modulus -> visibly flexible
hinge_pos = chrono.ChVector3d(0, 0, -0.1)                  # pinned (root) end A
free_pos = chrono.ChVector3d(cable_length, 0, -0.1)        # free end B (starts horizontal)

# === System & gravity === FEA uses ChSystemSMC; Y-up world with downward gravity
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Solver & timestepper === sparse-QR + linearized implicit Euler (ANCF cable)
solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# === FEA mesh & cable section === ChBeamSectionCable built into an ANCF beam chain
# FEA cable: no contact material needed — driven by the hinge constraint + gravity only.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

sec_cable = fea.ChBeamSectionCable()
sec_cable.SetDiameter(cable_diameter)
sec_cable.SetYoungModulus(cable_E)
sec_cable.SetRayleighDamping(0.000)

builder = fea.ChBuilderCableANCF()
builder.BuildBeam(mesh, sec_cable, n_elements, hinge_pos, free_pos)

# Keep a strong reference to the node container (SWIG GC pitfall) before indexing.
beam_nodes = builder.GetLastBeamNodes()                    # cache: container kept alive
root_node = beam_nodes.front()
tip_node = beam_nodes.back()

sys.Add(mesh)

# === Constraints === hinge the root node to a fixed ground/truss body (3 translational DOF)
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

hinge = fea.ChLinkNodeFrame()
hinge.Initialize(root_node, truss)
sys.Add(hinge)

# === FEA visualization === bending-moment surface field + node coordinate-system glyphs
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

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF cable beam hinged to ground under gravity")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)          # Y-up FEA world
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.2, -1.4), chrono.ChVector3d(0.5, -0.4, -0.1))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -0.5, -0.1), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === real-time render + ANCF time-stepping under gravity


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
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
    import traceback
    traceback.print_exc()
    raise
