"""
ANCF Cable Beam Simulation — PyChrono FEA demo.

Models a flexible cable composed of ANCF cable elements (ChBuilderCableANCF).
One end is pinned (hinged) to a fixed ground truss; the other end hangs freely
under gravity. The simulation visualises the deforming cable using
ChVisualShapeFEA coloured by nodal displacement norm (surface shape) plus node
coordinate-system glyphs, rendered in the Irrlicht window.

System: ChSystemSMC (required by FEA ground-truth demos).
Solver:  ChSolverSparseQR + ChTimestepperEulerImplicitLinearized (ANCF cable).
World convention: Y-up, gravity (0, -9.81, 0).
Contact: none — pure jointed FEA, no rigid-body collision surfaces.
"""

import os
import csv
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Constants ===
CABLE_LENGTH   = 0.5          # m — total cable length
CABLE_DIAM     = 0.015        # m — cable cross-section diameter
CABLE_DENSITY  = 1000.0       # kg/m³
CABLE_E        = 0.01e9       # Pa — Young's modulus (flexible cable)
CABLE_DAMPING  = 0.000        # Rayleigh damping
N_ELEMENTS     = 10           # number of ANCF cable elements

TIME_STEP      = 0.01         # s — ANCF cable canonical timestep
SIM_END        = 5.0          # s — simulation duration
RENDER_FPS     = 50.0         # Hz — Irrlicht frame rate
RENDER_EVERY   = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Cable start/end along the Y axis (Y-up: cable starts near origin, hangs down)
CABLE_A = chrono.ChVector3d(0.0, 0.0, -0.1)
CABLE_B = chrono.ChVector3d(CABLE_LENGTH, 0.0, -0.1)

# === System & gravity ===
# ChSystemSMC — required for all SimBench FEA demos.
# FEA beam: no collision system needed — pure jointed FEA, no rigid-body contact.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up convention

# Solver: SparseQR + EulerImplicitLinearized (ANCF cable canonical setup)
solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# === FEA mesh & ANCF cable ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)   # mesh applies gravity automatically

# ANCF cable section properties
sec_cable = fea.ChBeamSectionCable()
sec_cable.SetDiameter(CABLE_DIAM)
sec_cable.SetYoungModulus(CABLE_E)
sec_cable.SetRayleighDamping(CABLE_DAMPING)

# Build cable beam using ANCF builder
builder = fea.ChBuilderCableANCF()
builder.BuildBeam(mesh, sec_cable, N_ELEMENTS, CABLE_A, CABLE_B)

# SWIG GC PITFALL: store the node container before indexing (prevents dangling shared_ptr)
beam_nodes = builder.GetLastBeamNodes()
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]  # cache: strong refs to all nodes

# Fixed ground truss (anchor point for hinge)
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Hinge the FIRST node to the ground truss (one end pinned to ground)
# fea.ChLinkNodeFrame provides 3 translational DOF constraint (hinge in 9.0.0)
hinge = fea.ChLinkNodeFrame()
hinge.Initialize(nodes[0], truss)
sys.Add(hinge)

sys.Add(mesh)

# === FEA visualization shapes ===
# Shape 1 — surface coloured by nodal displacement norm
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
vis_surface.SetColorscaleMinMax(0.0, 0.4)   # two scalars (lo, hi)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node coordinate-system glyphs
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — full Irrlicht scene: window + sky + camera + lights ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up to match gravity
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono ANCF Cable Beam — FEA Demo")
vis.Initialize()                                     # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, -0.5, 0.3), chrono.ChVector3d(0.25, 0.0, -0.1))
vis.AddTypicalLights()

# === Review-only setup ===


# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            tip_pos = nodes[-1].GetPos()   # cache: reuse the pre-built list each step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
