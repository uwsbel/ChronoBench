import os
import errno

import pychrono.core      as chrono
import pychrono.irrlicht  as chronoirr
import pychrono.fea       as fea
import pychrono.pardisomkl as mkl





out_dir = os.path.join(chrono.GetChronoOutputPath(), "FEA_SHELLS_BST")
if not os.path.isdir(out_dir):
    try:
        os.mkdir(out_dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
            print("Error creating output directory:", out_dir)





sys  = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)


mesh.SetAutomaticGravity(False)





density   = 100.0           
E         = 6.0e4           
nu        = 0.0             
thickness = 0.01            


melasticity = fea.ChElasticityKirchhoffIsotropic(E, nu)
material    = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)





L_x          = 1.0
L_z          = 1.0
nsections_x  = 40
nsections_z  = 40

mynodes = []


def safe_node(idx):
    idx = max(0, min(idx, len(mynodes) - 1))
    return mynodes[idx]


for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        pos = chrono.ChVector3d(float(ix) * L_x / nsections_x,
                                0.0,
                                float(iz) * L_z / nsections_z)
        node = fea.ChNodeFEAxyz(pos)
        mesh.AddNode(node)
        mynodes.append(node)


melementmonitor = None  

for iz in range(nsections_z):
    for ix in range(nsections_x):

        
        elemA = fea.ChElementShellBST()
        mesh.AddElement(elemA)

        if iz == 0 and ix == 1:
            melementmonitor = elemA          

        n1 = safe_node( iz      * (nsections_x + 1) + ix     )
        n2 = safe_node( iz      * (nsections_x + 1) + ix + 1 )
        n3 = safe_node((iz + 1) * (nsections_x + 1) + ix     )

        
        n4 = safe_node((iz + 1) * (nsections_x + 1) + ix + 1)
        n5 = safe_node((iz + 1) * (nsections_x + 1) + max(ix - 1, 0))
        n6 = safe_node(max(iz - 1, 0) * (nsections_x + 1) + ix + 1)

        elemA.SetNodes(n1, n2, n3, n4, n5, n6)
        elemA.AddLayer(thickness, 0.0, material)

        
        elemB = fea.ChElementShellBST()
        mesh.AddElement(elemB)

        n1b = safe_node((iz + 1) * (nsections_x + 1) + ix + 1)
        n2b = safe_node((iz + 1) * (nsections_x + 1) + ix    )
        n3b = safe_node( iz      * (nsections_x + 1) + ix + 1)

        n4b = safe_node( iz      * (nsections_x + 1) + ix    )
        n5b = safe_node( iz      * (nsections_x + 1) + min(ix + 2, nsections_x))
        n6b = safe_node(min(iz + 2, nsections_z) * (nsections_x + 1) + ix)

        elemB.SetNodes(n1b, n2b, n3b, n4b, n5b, n6b)
        elemB.AddLayer(thickness, 0.0, material)


for j in range(30):
    for k in range(30):
        idx = j * (nsections_x + 1) + k
        if idx < len(mynodes):
            mynodes[idx].SetFixed(True)






mvisualizeshellA = fea.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetSmoothFaces(True)       
mvisualizeshellA.SetWireframe(True)         
mvisualizeshellA.SetBackfaceCull(True)      
mvisualizeshellA.SetShellResolution(2)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = fea.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Shells FEA test: triangle BST elements")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()





solver = mkl.ChSolverPardisoMKL()
solver.LockSparsityPattern(True)
sys.SetSolver(solver)





timestep = 0.005

sys.Setup()
sys.Update()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(timestep)