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
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Create elements
melementA = None
melementB = None
for iz in range(nsections_z):
    for ix in range(nsections_x):
        if iz == 0 and ix == 1:
            melementA = fea.ChElementShellBST()
            melementmonitor = melementA
        elif iz > 0 and ix > 0:
            melementA = fea.ChElementShellBST()
            melementB = fea.ChElementShellBST()
            melementB.SetNodes(mynodes[iz * (nsections_x + 1) + ix - 1], mynodes[iz * (nsections_x + 1) + ix],
                           mynodes[(iz + 1) * (nsections_x + 1) + ix], None, None)
            melementA.SetNodes(mynodes[iz * (nsections_x + 1) + ix], mynodes[iz * (nsections_x + 1) + ix + 1],
                           mynodes[(iz + 1) * (nsections_x + 1) + ix], None, None)
            melementA.AddLayer(thickness, 0, material)
            mesh.AddElement(melementA)

            melementB = fea.ChElementShellBST()
            melementB.SetNodes(mynodes[iz * (nsections_x + 1) + ix], mynodes[iz * (nsections_x + 1) + ix + 1],
                           mynodes[(iz + 1) * (nsections_x + 1) + ix], None, None)
            melementB.AddLayer(thickness, 0, material)
            mesh.AddElement(melementB)

# Create visualizations for shell elements
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetFaceType(chrono.ChVisualShapeFEA.FaceType_SMOOTH)
mvisualizeshellA.SetWireframe(False)
mvisualizeshellA.SetBackFaceCulling(True)
mvisualizeshellA.SetGlyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellA.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)

# Irrlicht visualization system setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# Change solver to PardisoMKL
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# Define time step for simulation
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

# Node monitoring and loading setup
nodePlotA = chrono.ChVisualElement(mvisualizeshellA)
nodePlotB = chrono.ChVisualElement(mvisualizeshellB)
nodesLoad = []
ref_X = chrono.ChVector3d(0, 0, 0)
ref_Y = chrono.ChVector3d(0, 0, 0)
load_force = chrono.ChVector3d(0, -1000, 0)

# Monitoring nodes and elements
mnodemonitor = chrono.ChNodeFEAxyz(mynodes[0])
melementmonitor = melementA

# Fix upper nodes
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# Adjust visualization shapes
# Uncomment the following lines for optional visualization adjustments
# mvisualizeshellA.SetFaceType(chrono.ChVisualShapeFEA.FaceType_SMOOTH)
# mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackFaceCulling(False)

# Solver adjustment
# No change needed as the solver is already set to PardisoMKL with LockSparsityPattern=True

# Time step adjustment
# No change needed as the time step is already set to 0.005