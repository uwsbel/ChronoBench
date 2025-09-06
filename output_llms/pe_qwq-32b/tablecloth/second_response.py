import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# Output directory setup
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# Create Chrono physical system
sys = chrono.ChSystemSMC()

# Create and add mesh to the system
mesh = fea.ChMesh()
sys.Add(mesh)

# Define node monitoring and loading setup
nodePlotA = None
nodePlotB = None
nodesLoad = []
load_force = chrono.ChVectorD(0, 0, 0)
mnodemonitor = None
melementmonitor = None

# Material properties
density = 100
E = 6e4
nu = 0.0
thickness = 0.01

# Create material
melasticity = fea.ChElasticityKirchhoffIsotropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# Mesh dimensions
L_x, L_z = 1, 1
nsections_x, nsections_z = 40, 40

# Create nodes
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVectorD(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)
        # Example monitoring node setup (adjust condition as needed)
        if iz == 0 and ix == 0:
            mnodemonitor = mnode

# Fix upper nodes
for j in range(30):
    for k in range(30):
        idx = j * (nsections_x + 1) + k
        mynodes[idx].SetFixed(True)

# Create elements
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Element A
        melementA = fea.ChElementShellBST()
        node1 = mynodes[iz * (nsections_x + 1) + ix]
        node2 = mynodes[iz * (nsections_x + 1) + ix + 1]
        node3 = mynodes[(iz + 1) * (nsections_x + 1) + ix]
        node4 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        melementA.SetNodes(node1, node2, node3, node4)
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)
        
        # Element B
        melementB = fea.ChElementShellBST()
        node1b = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        node2b = mynodes[(iz + 1) * (nsections_x + 1) + ix]
        node3b = mynodes[iz * (nsections_x + 1) + ix + 1]
        node4b = mynodes[iz * (nsections_x + 1) + ix]
        melementB.SetNodes(node1b, node2b, node3b, node4b)
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)
        
        # Element monitoring
        if iz == 0 and ix == 1:
            melementmonitor = melementA

# Visualization setup
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackfaceCulling(True)  # Optional, commented out
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Irrlicht visualization system setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 0.3, 1.3), chrono.ChVectorD(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# Adjust solver
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)  # Changed to True
sys.SetSolver(mkl_solver)

# Adjust time step
timestep = 0.005  # Changed from 0.001 to 0.005

# Final setup and system updates
sys.Setup()
sys.Update()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# Output directory setup
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# Create Chrono physical system
sys = chrono.ChSystemSMC()

# Define node monitoring and loading setup
nodePlotA = None
nodePlotB = None
nodesLoad = []
load_force = chrono.ChVectorD(0, 0, 0)
mnodemonitor = None
melementmonitor = None

# Material properties
density = 100
E = 6e4
nu = 0.0
thickness = 0.01

# Create material
melasticity = fea.ChElasticityKirchhoffIsotropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# Mesh dimensions
L_x, L_z = 1, 1
nsections_x, nsections_z = 40, 40

# Create nodes
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVectorD(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)
        # Example monitoring node setup (adjust condition as needed)
        if iz == 0 and ix == 0:
            mnodemonitor = mnode

# Fix upper nodes
for j in range(30):
    for k in range(30):
        idx = j * (nsections_x + 1) + k
        mynodes[idx].SetFixed(True)

# Create elements
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Element A
        melementA = fea.ChElementShellBST()
        node1 = mynodes[iz * (nsections_x + 1) + ix]
        node2 = mynodes[iz * (nsections_x + 1) + ix + 1]
        node3 = mynodes[(iz + 1) * (nsections_x + 1) + ix]
        node4 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        melementA.SetNodes(node1, node2, node3, node4)
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)
        
        # Element B
        melementB = fea.ChElementShellBST()
        node1b = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        node2b = mynodes[(iz + 1) * (nsections_x + 1) + ix]
        node3b = mynodes[iz * (nsections_x + 1) + ix + 1]
        node4b = mynodes[iz * (nsections_x + 1) + ix]
        melementB.SetNodes(node1b, node2b, node3b, node4b)
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)
        
        # Element monitoring
        if iz == 0 and ix == 1:
            melementmonitor = melementA

# Visualization setup
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackfaceCulling(True)  # Optional, commented out
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Irrlicht visualization system setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 0.3, 1.3), chrono.ChVectorD(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# Adjust solver
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)  # Changed to True
sys.SetSolver(mkl_solver)

# Adjust time step
timestep = 0.005  # Changed from 0.001 to 0.005

# Final setup and system updates
sys.Setup()
sys.Update()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)