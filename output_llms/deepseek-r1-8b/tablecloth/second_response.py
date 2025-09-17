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

# Material properties
density = 100
E = 6e4
nu = 0.0
thickness = 0.01

# Create material
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# Mesh dimensions
L_x, L_z = 1, 1
nsections_x, nsections_z = 40, 40

# Create nodes
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Fix certain nodes (based on instructions)
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# Create elements
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

# Node monitoring and loading setup (based on instructions)
nodePlotA = mynodes[0]
nodePlotB = mynodes[0]
nodesLoad = []

# Create interpolation functions for reference tracking
ref_X = lambda iz, ix: ix * (L_x / nsections_x)
ref_Y = lambda iz, ix: iz * (L_z / nsections_z)

# Define load force vector
load_force = chrono.ChVector3d(0, 0, 0)

# Add monitoring nodes and elements
mnodemonitor = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, 0))
mesh.AddNode(mnodemonitor)

# Assign monitoring to specific element (based on instructions)
ementmonitor = None
for iz in range(nsections_z):
    for ix in range(nsections_x):
        if iz == 0 and ix == 1:
            emelement = melementA
            emelementmonitor = emelement
            break
    if emelementmonitor:
        break

# Visualization adjustments (based on instructions)
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
# mvisualizeshellA.SetSmoothFaces(True)
# mvisualizeshellA.SetWireframeMode(True)
# mvisualizeshellA.SetBackfaceCulling(True)  # Commented out for optional adjustment
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Solver adjustment (based on instructions)
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# Time step adjustment (based on instructions)
timestep = 0.005

# Final setup and system updates
sys.Setup()
sys.Update()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)