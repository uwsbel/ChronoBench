"""
PyChrono FEA beam demonstration — Euler-Bernoulli cantilever beam under gravity.
A steel beam is clamped at its root (x=0) and sags under its own weight.
Uses ChSystemSMC + Pardiso MKL solver + HHT timestepper for stiff beam dynamics.
Visualization via Irrlicht with ChVisualShapeFEA (surface moment field + node glyphs).
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# review-only: sim_recording for frame capture + CSV + video assembly

# === Simulation parameters ===
time_step = 1e-3          # FEA stiff beam: 1e-3 s
sim_end = 5.0             # 5 seconds
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Beam geometry and material (steel cantilever)
beam_L = 4.0              # beam length [m]
n_elements = 20           # number of beam elements
diameter = 0.10           # circular cross-section diameter [m]
density = 7850.0          # steel density [kg/m³]
E = 210e9                 # Young's modulus [Pa]
nu = 0.3                  # Poisson's ratio (for deriving G)
rayleigh_damping = 0.0    # no damping for this demo

# === System creation (FEA uses ChSystemSMC + Y-up gravity) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up

# Direct solver required for stiff beam stiffness matrices
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper (canonical minimal form for stiff beams)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# FEA beam: no contact material needed — driven by gravity + constraints only

# === FEA mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Euler-Bernoulli beam section
sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(diameter)
sec.SetDensity(density)
sec.SetYoungModulus(E)
sec.SetShearModulusFromPoisson(nu)
sec.SetRayleighDamping(rayleigh_damping)

# Build the cantilever beam along the X axis
up = chrono.ChVector3d(0, 1, 0)  # section Y direction (lateral)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh, sec, n_elements,
    chrono.ChVector3d(0, 0, 0),         # root (fixed)
    chrono.ChVector3d(beam_L, 0, 0),     # tip (free)
    up,
)

# Fix the root node (cantilever clamp) — SWIG GC pitfall: keep strong reference
beam_nodes = builder.GetLastBeamNodes()
root_node = beam_nodes.front()
root_node.SetFixed(True)

# Access tip node for data logging
tip_node = beam_nodes.back()

sys.Add(mesh)

# === Visualization (full Irrlicht block) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA Beam — Euler-Bernoulli cantilever under gravity")
vis.Initialize()                                        # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -6, 4), chrono.ChVector3d(2, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# FEA visualization — two-shape pattern (surface + glyph)
# Shape 1: surface scalar field (beam bending moment Mz)
vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-0.5, 0.5)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)

# Shape 2: node coordinate-system glyphs
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.015)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Review-only: prepare frame capture + CSV ===

# === Main simulation loop ===
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break

# === Review-only: close CSV, assemble video, plot, cleanup ===
