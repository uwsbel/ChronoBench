import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os


out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)


density = 100
E = 6e4
nu = 0.0
thickness = 0.01


melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)


L_x, L_z = 1, 1
nsections_x, nsections_z = 40, 40


mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)


nodePlotA = mynodes[20 * (nsections_x + 1) + 20]  
nodePlotB = mynodes[10 * (nsections_x + 1) + 10]  
nodesLoad = [mynodes[iz * (nsections_x + 1) + nsections_x] for iz in range(nsections_z + 1)]  


def ref_X(node):
    return node.GetPos().x

def ref_Y(node):
    return node.GetPos().z


ementmonitor = None
for iz in range(nsections_z):
    for ix in range(nsections_x):
        
        melementA = fea.ChElementShellBST()
        node1 = mynodes[iz * (nsections_x + 1) + ix]
        node2 = mynodes[iz * (nsections_x + 1) + ix + 1]
        node3 = mynodes[(iz + 1) * (nsections_x + 1) + ix]
        node4 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        melementA.SetNodes(node1, node2, node3, node4)
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        
        if iz == 0 and ix == 1:
            ementmonitor = melementA

        
        melementB = fea.ChElementShellBST()
        node1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        node2 = mynodes[(iz + 1) * (nsections_x + 1) + ix]
        node3 = mynodes[iz * (nsections_x + 1) + ix + 1]
        node4 = mynodes[iz * (nsections_x + 1) + ix + 2]
        melementB.SetNodes(node1, node2, node3, node4)
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)


for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)


mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
mvisualizeshellA.SetBackfaceCulling(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetDataType(chrono.ChVisualShapeFEA.DataType_NONE)
mesh.AddVisualShapeFEA(mvisualizeshellB)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()


mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)


timestep = 0.005


sys.Setup()
sys.Update()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)