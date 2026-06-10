"""
Cable simulation using ANCF cable elements with gravity and an applied tip force.

This demo models a flexible cable under gravity with a downward tip load.
The cable is anchored at the left end to a fixed truss. Uses ChSystemSMC
with ANCF cable elements, sparse QR solver (changed to MINRES per iteration 2
request), and Euler implicit linearized timestepper.

Changes from iteration 1:
  - Rayleigh damping: 0.000 -> 0.0001
  - Applied force: (0, -0.2, 0) -> (0, -0.7, 0)
  - Solver: ChSolverSparseQR -> ChSolverMINRES with tolerance 1e-10,
    max_iter 200, diagonal preconditioner, warm start, verbose False
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

import sim_recording as rec

# === Simulation parameters ===
time_step = 0.01          # ANCF cable timestep (larger than rigid body)
sim_end = 3.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Cable geometry
cable_L = 1.0             # cable length (m)
n_elements = 10           # number of cable elements
diameter = 0.015          # cable diameter (m)
young_modulus = 0.01e9   # Young's modulus (Pa) — soft rubber-like
rayleigh_damping = 0.0001  # Rayleigh damping coefficient (changed from 0.000)
density = 1000.0          # kg/m^3

# Applied load
tip_force = chrono.ChVector3d(0, -0.7, 0)  # changed from (0, -0.2, 0)

# === System setup (SMC for FEA) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# MINRES solver with the requested settings
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)
sys.SetSolver(solver)

print("Using MINRES solver")

# Timestepper: Euler implicit linearized for ANCF cable
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# === FEA Mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Cable section
sec_cable = fea.ChBeamSectionCable()
sec_cable.SetDiameter(diameter)
sec_cable.SetYoungModulus(young_modulus)
sec_cable.SetRayleighDamping(rayleigh_damping)
sec_cable.SetDensity(density)

# Build the cable using ANCF builder
builder = fea.ChBuilderCableANCF()
builder.BuildBeam(
    mesh, sec_cable, n_elements,
    chrono.ChVector3d(0, 0, -0.1),    # start point A
    chrono.ChVector3d(cable_L, 0, -0.1)  # end point B
)

# Keep a reference to prevent SWIG GC
beam_nodes = builder.GetLastBeamNodes()
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]

# === Anchor the left end to a fixed truss ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Hinge the cable start node to the truss (3 translational DOF released)
anchor_hinge = fea.ChLinkNodeFrame()
anchor_hinge.Initialize(nodes[0], truss)
sys.Add(anchor_hinge)

# === Applied force on the front (tip) node ===
tip_node = nodes[-1]
tip_node.SetForce(tip_force)

# Register mesh
sys.Add(mesh)

# === Visualization (full Irrlicht block) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable — tip force demo")
vis.Initialize()

# Logo, sky, camera, lights AFTER Initialize
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, -0.5, 1.0), chrono.ChVector3d(0.5, 0, -0.1))
vis.AddTypicalLights()

# Add ground reference grid
vis.AddGrid(
    0.1, 0.1, 40, 40,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.12), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4)
)

# FEA visualization: surface field + node glyphs
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
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

# === CSV logging setup (review-only) ===
csv_file = "simulation_data.csv"

# === Main simulation loop ===
frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # review-only: capture frame

    # Inner physics batch
    for _ in range(render_every):
        sim_time = sys.GetChTime()

        # review-only: log CSV each physics step

        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break

# review-only: close CSV
