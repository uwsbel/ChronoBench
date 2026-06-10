"""Tablecloth folding with Kirchhoff thin-shell (BST) finite elements.

Models a square tablecloth as a triangular Kirchhoff-Love thin-shell mesh built
from `ChElementShellBST` elements over a regular grid of `ChNodeFEAxyz` nodes.
The material is an isotropic Kirchhoff elasticity (`ChElasticityKirchhoffIsothropic`)
wrapped in a `ChMaterialShellKirchhoff`. The deformable mesh lives in a
`ChSystemSMC` smooth-contact system and is integrated with the direct PardisoMKL
sparse solver, which is required for the stiff shell stiffness matrices (iterative
solvers diverge). Under gravity the initially flat cloth sags, drapes and folds.
World is Y-up: the cloth lies in the X-Z plane at y=0 and falls along -Y.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

# === Parameters === cloth geometry, material, and time stepping
L_x, L_z = 1.0, 1.0                 # tablecloth side lengths (m), X-Z plane
nsections_x, nsections_z = 40, 40   # grid subdivisions per side -> BST triangle pairs
density = 100.0                     # areal/volumetric density (kg/m^3)
E = 6e4                             # Young's modulus (Pa) — light fabric
nu = 0.0                            # Poisson ratio (isotropic Kirchhoff)
thickness = 0.01                    # shell layer thickness (m)
time_step = 1e-3                    # stiff shell -> small step
sim_end = 2.0                       # simulation duration (s)
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# === System & gravity === SMC smooth-contact system for FEA shells (Y-up world)
sys = chrono.ChSystemSMC()

# === Mesh & material === isotropic Kirchhoff thin-shell, registered with the system
mesh = fea.ChMesh()
sys.Add(mesh)

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)   # isotropic Kirchhoff law
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# === Nodes === regular (nsections+1)^2 grid of xyz nodes in the X-Z plane
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0.0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# === Elements === two BST triangles per grid cell with their neighbour stencil
for iz in range(nsections_z):
    for ix in range(nsections_x):
        melementA = fea.ChElementShellBST()
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else None
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else None
        melementA.SetNodes(mynodes[iz * (nsections_x + 1) + ix],
                           mynodes[iz * (nsections_x + 1) + ix + 1],
                           mynodes[(iz + 1) * (nsections_x + 1) + ix],
                           boundary_1, boundary_2, boundary_3)
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        melementB = fea.ChElementShellBST()
        boundary_1 = mynodes[iz * (nsections_x + 1) + ix]
        boundary_2 = mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None
        melementB.SetNodes(mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
                           mynodes[(iz + 1) * (nsections_x + 1) + ix],
                           mynodes[iz * (nsections_x + 1) + ix + 1],
                           boundary_1, boundary_2, boundary_3)
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)

# === FEA visualization === coloured shell surface + node-dot glyphs (attach before Initialize)
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth folding: Kirchhoff BST shell elements")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# === Solver === direct PardisoMKL sparse solver (required for stiff shell matrices)
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(False)
sys.SetSolver(mkl_solver)

# Shell meshes require an explicit setup/update pass before stepping
sys.Setup()
sys.Update()

# === Main loop === render at a fixed cadence, advance shell dynamics each step

center_node = mynodes[(nsections_z // 2) * (nsections_x + 1) + nsections_x // 2]  # cache: mid node
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
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble the review video and timeseries plot
