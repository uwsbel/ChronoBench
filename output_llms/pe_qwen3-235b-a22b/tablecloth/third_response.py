import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# Output directory
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"

# Create (if needed) the output directory
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

# Create a mesh
mesh = fea.ChMesh()
sys.Add(mesh)

# Disable gravity
mesh.SetAutomaticGravity(False)

# Monitoring nodes and elements
mnodemonitor = fea.ChNodeFEAxyz()
melementmonitor = fea.ChElementShellBST()

if True:
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
    L_x = 1
    nsections_x = 40
    L_z = 1
    nsections_z = 40

    # Create nodes
    mynodes = []
    for iz in range(nsections_z + 1):
        for ix in range(nsections_x + 1):
            p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
            mnode = fea.ChNodeFEAxyz(p)
            mesh.AddNode(mnode)
            mynodes.append(mnode)

    # Create elements
    for iz in range(nsections_z):
        for ix in range(nsections_x):
            # First element
            melementA = fea.ChElementShellBST()
            mesh.AddElement(melementA)
            
            if iz == 0 and ix == 1:
                melementmonitor = melementA  # Fixed typo here

            # Set nodes for element A (only 3 main nodes)
            melementA.SetNodes(
                mynodes[iz * (nsections_x + 1) + ix],
                mynodes[iz * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix]
            )
            
            melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

            # Second element
            melementB = fea.ChElementShellBST()
            mesh.AddElement(melementB)
            
            # Set nodes for element B (only 3 main nodes)
            melementB.SetNodes(
                mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                mynodes[iz * (nsections_x + 1) + ix + 1]
            )
            
            melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

    # Fix boundary nodes
    for j in range(30):
        for k in range(30):
            mynodes[j * (nsections_x + 1) + k].SetFixed(True)

    # Visualization settings - Enhanced
    mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
    mvisualizeshellA.SetSmoothFaces(True)  # Enabled smooth faces
    mvisualizeshellA.SetWireframe(True)    # Enabled wireframe mode
    mvisualizeshellA.SetShellResolution(2)
    mvisualizeshellA.SetBackfaceCull(True) # Enabled backface culling
    mesh.AddVisualShapeFEA(mvisualizeshellA)

    mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
    mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    mvisualizeshellB.SetSymbolsThickness(0.006)
    mesh.AddVisualShapeFEA(mvisualizeshellB)

# Visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))
vis.AddTypicalLights()

# Solver setup
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# Simulation loop
timestep = 0.005
sys.Setup()
sys.Update()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)

mvisualizeshellA.SetSmoothFaces(True)
     mvisualizeshellA.SetWireframe(True)
     mvisualizeshellA.SetBackfaceCull(True)