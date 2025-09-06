import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# Output directory
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory ")

# Create physical system
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)
mesh.SetAutomaticGravity(False)

# Define material properties
density = 100
E = 6e4
nu = 0.0
thickness = 0.01

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

# Create elements with correct boundary nodes
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Element A
        melementA = fea.ChElementShellBST()
        mesh.AddElement(melementA)
        
        # CORRECTED: Adjacent nodes for boundaries
        nodeA = mynodes[iz * (nsections_x + 1) + ix]
        nodeB = mynodes[iz * (nsections_x + 1) + ix + 1]
        nodeC = mynodes[(iz + 1) * (nsections_x + 1) + ix]
        
        # Boundary nodes (adjacent to edges)
        bndA = mynodes[iz * (nsections_x + 1) + ix + 1] if ix < nsections_x - 1 else None
        bndB = mynodes[(iz + 1) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None
        bndC = mynodes[iz * (nsections_x + 1) + ix] if ix > 0 else None
        
        melementA.SetNodes(nodeA, nodeB, nodeC, bndA, bndB, bndC)
        melementA.AddLayer(thickness, 0, material)

        # Element B
        melementB = fea.ChElementShellBST()
        mesh.AddElement(melementB)
        
        nodeD = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        melementB.SetNodes(nodeB, nodeD, nodeC, 
                          mynodes[(iz + 1) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None,
                          mynodes[iz * (nsections_x + 1) + ix + 1] if ix < nsections_x - 1 else None,
                          mynodes[(iz + 1) * (nsections_x + 1) + ix + 1] if iz < nsections_z - 1 else None)
        melementB.AddLayer(thickness, 0, material)

# CORRECTED: Fix nodes using mesh dimensions
for iz in range(min(30, nsections_z + 1)):
    for ix in range(min(30, nsections_x + 1)):
        idx = iz * (nsections_x + 1) + ix
        if idx < len(mynodes):
            mynodes[idx].SetFixed(True)

# Visualization with requested features
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetSmoothFaces(True)  # ENABLED SMOOTH FACES
mvisualizeshellA.SetWireframe(True)    # ENABLED WIREFRAME
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetBackfaceCull(True) # ENABLED BACKFACE CULLING
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

# Solver
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# Simulation setup
timestep = 0.005
sys.Setup()
sys.Update()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)