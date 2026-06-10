"""Tablecloth draping — Kirchhoff BST triangular shell FEA (SMC system).

Models a 1 m x 1 m thin cloth as a regular grid of ChNodeFEAxyz nodes meshed
with paired ChElementShellBST triangles (the BST stencil carries three extra
neighbour nodes per triangle for bending). The cloth uses an isotropic Kirchhoff
shell material; a 30x30 block of upper-corner nodes is held fixed so the rest of
the sheet drapes under gravity. Monitoring nodes/elements and reference-tracking
interpolation functions are wired for diagnostics. Solved with a direct Pardiso
MKL solver (sparsity pattern locked) at a 0.005 s timestep.

Pure FEA shell scene: no rigid-body collision/contact, so no collision system is
needed (the cloth nodes are not given a contact surface here).
Expected behavior: the unfixed portion of the cloth sags/drapes under gravity.
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import os

# === Parameters (geometry / material / time) ===
density = 100        # cloth material density (kg/m^3)
E = 6e4             # Young's modulus (Pa)
nu = 0.0            # Poisson's ratio
thickness = 0.01    # shell thickness (m)
L_x = 1             # cloth length in x (m)
nsections_x = 40    # element divisions in x
L_z = 1             # cloth length in z (m)
nsections_z = 40    # element divisions in z
timestep = 0.005    # implicit integration step (s)
sim_end = 4.0       # bounded recording horizon (s)

# === System & mesh ===  SMC system with an FEA mesh container; gravity on
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)
mesh.SetAutomaticGravity(True)   # cloth drapes under gravity

# === Diagnostics (monitoring nodes / elements + reference tracking) ===
nodePlotA = fea.ChNodeFEAxyz()           # node earmarked for plotting
nodePlotB = fea.ChNodeFEAxyz()           # second node earmarked for plotting
nodesLoad = []                           # nodes that could receive an external load
ref_X = chrono.ChFunctionInterp()        # reference-tracking interpolation (x)
ref_Y = chrono.ChFunctionInterp()        # reference-tracking interpolation (y)
load_force = chrono.ChVector3d()         # load force vector placeholder
mnodemonitor = fea.ChNodeFEAxyz()        # node for monitoring
melementmonitor = fea.ChElementShellBST()  # element for monitoring

# === Material (isotropic Kirchhoff shell) ===
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# === Nodes (regular grid in the x-z plane) ===
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# === Elements (paired BST triangles per grid cell, with bending neighbours) ===
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # First triangle of the cell
        melementA = fea.ChElementShellBST()
        mesh.AddElement(melementA)

        if (iz == 0 and ix == 1):
            ementmonitor = melementA   # earmark one element for monitoring

        # Boundary (bending) neighbours, guarded against grid edges
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if (ix > 0) else None
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if (iz > 0) else None

        melementA.SetNodes(
            mynodes[(iz) * (nsections_x + 1) + ix],
            mynodes[(iz) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1, boundary_2, boundary_3,
        )
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

        # Second triangle of the cell
        melementB = fea.ChElementShellBST()
        mesh.AddElement(melementB)

        boundary_1 = mynodes[(iz) * (nsections_x + 1) + ix]
        boundary_2 = mynodes[(iz) * (nsections_x + 1) + ix + 2] if (ix < nsections_x - 1) else None
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if (iz < nsections_z - 1) else None

        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[(iz) * (nsections_x + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3,
        )
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

# === Boundary conditions (hold a 30x30 corner block fixed) ===
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# === FEA visualization shapes (surface + node-dot glyphs) ===
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))
vis.AddTypicalLights()

# === Solver (direct Pardiso MKL; sparsity pattern locked for speed) ===
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# === Final setup (required for shells before stepping) ===
sys.Setup()
sys.Update()

rec_X = chrono.ChFunctionInterp()        # records for reference tracking (x)
rec_Y = chrono.ChFunctionInterp()        # records for reference tracking (y)

# === Render cadence (one render per frame, batch physics between frames) ===
render_fps = 30.0                                          # precomputed once
render_every = max(1, round(1.0 / (render_fps * timestep)))  # precomputed once


# === Main loop === render per frame, batch physics steps between frames
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(timestep)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:    # solver divergence / bad FEA state
    import traceback
    traceback.print_exc()
    raise
