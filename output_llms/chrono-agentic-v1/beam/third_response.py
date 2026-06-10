"""
Two-segment Euler-Bernoulli cantilever beam simulation using PyChrono FEA.

System type: ChSystemSMC (required for FEA stiff beams with Pardiso MKL solver).
Structure:
  - Beam segment 1: from point A=(0,0,0) to point B=(0.2,0,0), clamped at A.
  - Beam segment 2: from the last node of segment 1 (at 0.2,0,0) to
    point (0.2, 0.1, -0.1), Y-up direction (0,1,0).
    Segment 2 is built using the exact last node of segment 1 as its start,
    ensuring a continuous, structurally connected two-segment beam.

Expected behavior: the L-shaped two-segment beam deflects under gravity,
with the angled second segment amplifying displacement at its free tip.

FEA beam: no contact material needed — driven by constraints + gravity only.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Constants ===
BEAM_DIAMETER   = 0.012        # m — circular cross-section diameter
BEAM_DENSITY    = 7800.0       # kg/m³ — steel
YOUNG_MODULUS   = 210e9        # Pa
POISSON_RATIO   = 0.3
RAYLEIGH_DAMP   = 0.01
N_ELEMENTS_1    = 6            # elements in segment 1
N_ELEMENTS_2    = 6            # elements in segment 2

# Geometry of segment 2 (segment 1 ends at (0.2,0,0) by definition of segment 2 start)
PT_A1 = chrono.ChVector3d(0.0,  0.0,  0.0)   # start of segment 1 (clamped root)
PT_B1 = chrono.ChVector3d(0.2,  0.0,  0.0)   # end of segment 1 / start of segment 2
PT_B2 = chrono.ChVector3d(0.2,  0.1, -0.1)   # 'B' point of segment 2
UP_DIR = chrono.ChVector3d(0.0,  1.0,  0.0)  # Y-up for both segments

TIME_STEP   = 0.001   # s — stiff Euler beams require fine timestep
SIM_END     = 5.0     # s
RENDER_FPS  = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
# ChSystemSMC + Pardiso MKL is required for stiff FEA beam elements.
# World is Y-up for FEA (gravity along -Y). No rigid-body contact in this scene.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))

solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)

ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA Mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# -- Shared beam cross-section for both segments --
sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(BEAM_DIAMETER)
sec.SetDensity(BEAM_DENSITY)
sec.SetYoungModulus(YOUNG_MODULUS)
sec.SetShearModulusFromPoisson(POISSON_RATIO)
sec.SetRayleighDamping(RAYLEIGH_DAMP)

# -- Segment 1: from PT_A1 to PT_B1 (horizontal) --
builder1 = fea.ChBuilderBeamEuler()
builder1.BuildBeam(mesh, sec, N_ELEMENTS_1, PT_A1, PT_B1, UP_DIR)

# Fix the root node of segment 1 (clamped boundary condition at origin)
# SWIG GC safety: store container before indexing nodes
beam1_nodes = builder1.GetLastBeamNodes()
beam1_list  = [beam1_nodes[i] for i in range(beam1_nodes.size())]
beam1_list[0].SetFixed(True)   # front = start node (clamped root)

# The last node of segment 1 is used directly as the 'A' node of segment 2.
# Using the overload: BuildBeam(mesh, sec, N, nodeA, B_point, Ydir)
# This ensures the junction is a single shared DOF — no extra weld constraint needed.
node_junction = beam1_list[-1]  # cache: last node of seg 1, also first node of seg 2

# -- Segment 2: starts from node_junction (shared node), ends at PT_B2 --
builder2 = fea.ChBuilderBeamEuler()
builder2.BuildBeam(mesh, sec, N_ELEMENTS_2, node_junction, PT_B2, UP_DIR)

# SWIG GC safety: store container before indexing
beam2_nodes = builder2.GetLastBeamNodes()
beam2_list  = [beam2_nodes[i] for i in range(beam2_nodes.size())]

# Cache tip node (free end of segment 2) for position logging
tip_node = beam2_list[-1]  # cache: tip of the two-segment beam

# Register mesh with system
sys.Add(mesh)

# === FEA Visualization ===
# Attach visual shapes to mesh BEFORE vis.Initialize() so Irrlicht picks them up.
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.1, 0.1)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.003)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — Irrlicht window ===
# FEA scene is Y-up; SetCameraVertical matches gravity convention.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Two-Segment Euler-Bernoulli Beam — PyChrono FEA")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                     # Initialize FIRST (Irrlicht order: inverse of VSG)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.4, 0.3, 0.4), chrono.ChVector3d(0.1, 0.05, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.05, 0.05, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0.0, -0.05, 0.0), chrono.QUNIT),
            chrono.ChColor(0.45, 0.45, 0.45))

# === Review-only setup ===

# === Main simulation loop ===
frame = 0
try:
        while vis.Run() and sys.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(RENDER_EVERY):
                t        = sys.GetChTime()
                tip_pos  = tip_node.GetPos()       # cache: retrieved once per step
                junc_pos = node_junction.GetPos()  # cache: retrieved once per step
                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= SIM_END:
                    break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
