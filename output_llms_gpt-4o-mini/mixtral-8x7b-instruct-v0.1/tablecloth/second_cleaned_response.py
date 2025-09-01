import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

Create node variables for plotting and loading setup
=====================================================

nodePlotA, nodePlotB = None, None
nodesLoad = []

Create interpolation functions for reference tracking
==================================================

ref\_X, ref\_Y = 0.5, 0.5

Introduce load force vector
==========================

load\_force = [0, -1000, 0]

Create monitoring nodes and elements
===================================

mnodemonitor = None
melementmonitor = None

Create the Chrono system and add mesh
=====================================

sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)

Define material properties
==========================

density = 100
E = 6e4
nu = 0.0
thickness = 0.01

Create material
=============

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

Set up mesh dimensions and create nodes
========================================

L\_x, L\_z = 1, 1
nsections\_x, nsections\_z = 40, 40

mynodes = []
for iz in range(nsections\_z + 1):
for ix in range(nsections\_x + 1):
p = chrono.ChVector3d(ix \* (L\_x / nsections\_x), 0, iz \* (L\_z / nsections\_z))
mnode = fea.ChNodeFEAxyz(p)
mesh.AddNode(mnode)
mynodes.append(mnode)
if iz == nsections\_z and ix == 1:
melementmonitor = melementA
if iz == 0 and ix == 1:
nodePlotA = mnode
nodesLoad.append(mnode)
if ix == nsections\_x and iz == nsections\_z:
nodePlotB = mnode

Construct boundary nodes with conditional checks
=============================================

for iz in range(nsections\_z):
for ix in range(nsections\_x):
melementA = fea.ChElementShellBST()
boundary\_1 = mynodes[(iz + 1) \* (nsections\_x + 1) + ix + 1] \
if (ix > 0) else None
boundary\_2 = mynodes[(iz + 1) \* (nsections\_x + 1) + ix - 1] \
if (ix > 0 and iz > 0) else None
boundary\_3 = mynodes[(iz - 1) \* (nsections\_x + 1) + ix + 1] \
if (iz > 0) else None

melementA.SetNodes(mynodes[iz \* (nsections\_x + 1) + ix], mynodes[iz \* (nsections\_x + 1) + ix + 1],
mynodes[(iz + 1) \* (nsections\_x + 1) + ix], boundary\_1, boundary\_2, boundary\_3)
melementA.AddLayer(thickness, 0, material)
mesh.AddElement(melementA)

melementB = fea.ChElementShellBST()
boundary\_1 = mynodes[iz \* (nsections\_x + 1) + ix]
boundary\_2 = mynodes[iz \* (nsections\_x + 1) + ix + 2] \
if (ix < nsections\_x - 1) else None
boundary\_3 = mynodes[(iz + 2) \* (nsections\_x + 1) + ix] \
if (iz < nsections\_z - 1) else None

melementB.SetNodes(mynodes[(iz + 1) \* (nsections\_x + 1) + ix + 1], mynodes[(iz + 1) \* (nsections\_x + 1) + ix],
mynodes[iz \* (nsections\_x + 1) + ix + 1], boundary\_1, boundary\_2, boundary\_3)
melementB.AddLayer(thickness, 0, material)
mesh.AddElement(melementB)

Fix upper nodes
==============

for j in range(30):
for k in range(30):
mynodes[j \* (nsections\_x + 1) + k].SetFixed(True)

Adjust visualization shapes
==========================

mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetWireframe(True)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetBackfaceCulling(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType\_NODE\_DOT\_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType\_NONE)
mesh.AddVisualShapeFEA(mvisualizeshellB)

Set up Irrlicht visualization system
===================================

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo\_pychrono\_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

Adjust solver settings
=====================

mkl\_solver = mkl.ChSolverPardisoMKL()
mkl\_solver.LockSparsityPattern(True)
sys.SetSolver(mkl\_solver)

Adjust time step
===============

timestep = 0.005

Final setup and system updates
=============================

sys.Setup()
sys.Update()

Simulation loop
==============

while vis.Run():
vis.BeginScene()
vis.Render()
vis.EndScene()
sys.DoStepDynamics(timestep)

Apply loads
==========

nodePlotA.AddForce(load\_force)

Monitor nodes and elements
=========================

mnodemonitor = nodePlotA
melementmonitor = melementA

Plot results
============