import os
import math
import numpy as np
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", "data"))

# Initialize the system with SMC contact and PardisoMKL solver
sys = chrono.ChSystemSMC()
sys.SetSolverType(chrono.ChSolver.Type_PARDISOMKL)

# Create FEA mesh for the tablecloth
mesh = fea.ChMesh()

# Define material properties for isotropic Kirchhoff shell
young_modulus = 2e5  # Pa (example value for cloth-like material)
poisson_ratio = 0.3
thickness = 0.001  # 1 mm
density = 1000  # kg/m^3
material = fea.ChMaterialShellKirchhoff(young_modulus, poisson_ratio, thickness, density)

# Generate grid of nodes for the tablecloth
nx = 20  # Number of elements in x-direction
ny = 20  # Number of elements in y-direction
length = 1.0  # Tablecloth length (m)
width = 1.0   # Tablecloth width (m)
dx = length / nx
dy = width / ny

nodes = []
for i in range(nx + 1):
    row = []
    for j in range(ny + 1):
        x = i * dx - length / 2
        z = j * dy - width / 2
        y = 0.0  # Tablecloth lies in X-Z plane
        node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(x, y, z)))
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)

# Create shell elements connecting the nodes
for i in range(nx):
    for j in range(ny):
        n1 = nodes[i][j]
        n2 = nodes[i+1][j]
        n3 = nodes[i+1][j+1]
        n4 = nodes[i][j+1]
        element = fea.ChElementShellKirchhoff()
        element.SetNodes(n1, n2, n3, n4)
        element.SetMaterial(material)
        mesh.AddElement(element)

# Fix corner nodes to simulate holding the tablecloth
nodes[0][0].SetFixed(True)
nodes[nx][0].SetFixed(True)
nodes[nx][ny].SetFixed(True)
nodes[0][ny].SetFixed(True)

# Add the mesh to the system
sys.Add(mesh)

# Add visualization assets for the FEA mesh
vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_FEM_DATATYPE_NONE)
vis_mesh.SetWireframe(True)
vis_mesh.SetDefaultMeshColor(chrono.ChColor(0.5, 0.5, 1.0))  # Light blue wireframe
mesh.AddAsset(vis_mesh)

# Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.5, 1.5))  # Camera position
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)