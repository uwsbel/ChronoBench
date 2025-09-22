#!/usr/bin/env python3
# ======================================================================
# Table-cloth folding with Kirchhoff shell elements (PyChrono example)
# ======================================================================

import pychrono as chrono
import pychrono.irrlicht as irr

# ----------------------------------------------------------------------
# 1. Chrono system, solver and basic settings
# ----------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())       # use default data folder
sys = chrono.ChSystemSMC()                                 # SMC (penalty) contact model
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))               # gravity

# PardisoMKL sparse direct solver (very efficient for shell meshes!)
solver = chrono.ChSolverPardisoMKL()
solver LockSparsityPattern = True                          # reuse sparsity between steps
sys.SetSolver(solver)

# ----------------------------------------------------------------------
# 2. Mesh: build a regular grid of Kirchhoff shell elements
# ----------------------------------------------------------------------
mesh = chrono.ChMesh()

# -------------- material ------------------------------------------------
E_mod      = 1.0e7        # Young modulus [Pa]
nu         = 0.3          # Poisson ratio
rho        = 500          # density  [kg/m³]
thickness  = 0.002        # plate thickness [m]

mat = chrono.ChMaterialShellKirchhoff(rho, E_mod, nu)      # isotropic Kirchhoff law

# -------------- geometry ------------------------------------------------
Nx   = 30             # number of elements in X
Ny   = 30             # number of elements in Y
Lx   = 0.60           # total length in X [m]
Ly   = 0.60           # total length in Y [m]

dx   = Lx / Nx
dy   = Ly / Ny

# grids of node references to simplify element creation
nodes = [[None]*(Ny+1) for _ in range(Nx+1)]

# create nodes -----------------------------------------------------------
for ix in range(Nx+1):
    for iy in range(Ny+1):
        x = 0.0 + ix*dx
        y = 0.0                                      # initially flat in the X-Z plane
        z = 0.0 + iy*dy
        # set a slight downward offset on the free side so gravity starts the folding
        if iy == Ny:
            y = -0.05                                # 5 cm drop on the far edge

        node = chrono.ChNodeFEAxyzrot(chrono.ChVectorD(x, y, z), chrono.ChQuaternionD(1,0,0,0))
        node.SetMass(rho*thickness*dx*dy)            # lumped translational mass
        mesh.AddNode(node)
        nodes[ix][iy] = node

# fix the first row of nodes (ix = 0) to emulate the “clamped” table edge
for iy in range(Ny+1):
    nodes[0][iy].SetFixed(True)

# create elements --------------------------------------------------------
for ix in range(Nx):
    for iy in range(Ny):
        node00 = nodes[ix  ][iy  ]
        node10 = nodes[ix+1][iy  ]
        node11 = nodes[ix+1][iy+1]
        node01 = nodes[ix  ][iy+1]

        element = chrono.ChElementShellKirchhoff()
        element.SetNodes(node00, node10, node11, node01)
        element.SetMaterial(mat)
        element.SetThickness(thickness)
        element.SetAlphaDamp(0.04)                   # structural Rayleigh damping
        mesh.AddElement(element)

mesh.SetAutomaticGravity(False)                      # we already told the system to use gravity
sys.Add(mesh)

# ----------------------------------------------------------------------
# 3. Visualization with Irrlicht
# ----------------------------------------------------------------------
app = irr.ChIrrApp(sys, "Kirchhoff Shell Table-cloth Folding", irr.dimension2du(1280, 720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights(irr.vector3df( 2,  4,  2),
                     irr.vector3df(-2,  4, -2),
                     160.0, 2.0)
app.AddTypicalCamera(irr.vector3df(-0.2, 0.4, 0.8),
                     irr.vector3df( 0.3, 0.0, 0.3))

# color-coded Von-Mises stress plot for the shell mesh
vis_mesh = chrono.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMDataType(chrono.ChVisualizationFEAmesh.E_PLOT_SURFACE)
vis_mesh.SetColorScaleMinMax(0, 1e5)     # adjust scale to taste
vis_mesh.SetSmoothFaces(True)
vis_mesh.SetWireframe(False)
mesh.AddAsset(vis_mesh)

app.AssetBindAll()
app.AssetUpdateAll()

# ----------------------------------------------------------------------
# 4. Time-integration loop
# ----------------------------------------------------------------------
TIME_STEP = 1e-3
app.SetTimestep(TIME_STEP)

print("Running simulation.  Press Esc in the visualization window to quit.")

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()