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
        print("Error creating output directory ")


sys = chrono.ChSystemSMC()  


mesh = fea.ChMesh()  


sys.Add(mesh)


mesh.SetAutomaticGravity(False)


nodePlotA = fea.ChNodeFEAxyz()  
nodePlotB = fea.ChNodeFEAxyz()  
nodesLoad = []  


ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()


load_force = chrono.ChVector3d()


mnodemonitor = fea.ChNodeFEAxyz()  
melementmonitor = fea.ChElementShellBST()  

if (True):  
    
    density = 100  
    E = 6e4  
    nu = 0.0  
    thickness = 0.01  

    
    melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)

    
    material = fea.ChMaterialShellKirchhoff(melasticity)
    material.SetDensity(density)  

    
    L_x = 1  
    nsections_x = 40  
    L_z = 1  
    nsections_z = 40  

    
    mynodes = []

    
    for iz in range(nsections_z + 1):
        for ix in range(nsections_x + 1):
            p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))  
            mnode = fea.ChNodeFEAxyz(p)  
            mesh.AddNode(mnode)  
            mynodes.append(mnode)  

    
    for iz in range(nsections_z):
        for ix in range(nsections_x):
            
            melementA = fea.ChElementShellBST()
            mesh.AddElement(melementA)

            if (iz == 0 and ix == 1):
                ementmonitor = melementA  

            
            boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
            boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if (ix > 0) else None
            boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if (iz > 0) else None

            
            melementA.SetNodes(
                mynodes[(iz) * (nsections_x + 1) + ix],
                mynodes[(iz) * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                boundary_1, boundary_2, boundary_3
            )

            
            melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

            
            melementB = fea.ChElementShellBST()
            mesh.AddElement(melementB)

            
            boundary_1 = mynodes[(iz) * (nsections_x + 1) + ix]
            boundary_2 = mynodes[(iz) * (nsections_x + 1) + ix + 2] if (ix < nsections_x - 1) else None
            boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if (iz < nsections_z - 1) else None

            
            melementB.SetNodes(
                mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                mynodes[(iz) * (nsections_x + 1) + ix + 1],
                boundary_1, boundary_2, boundary_3
            )

            
            melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

    
    for j in range(30):
        for k in range(30):
            mynodes[j * (nsections_x + 1) + k].SetFixed(True)

    
    mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
    mvisualizeshellA.SetSmoothFaces(True)  
    mvisualizeshellA.SetWireframe(True)  
    mvisualizeshellA.SetBackfaceCull(True)  
    mvisualizeshellA.SetShellResolution(2)  
    mesh.AddVisualShapeFEA(mvisualizeshellA)  

    
    mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
    mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
    mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
    mvisualizeshellB.SetSymbolsThickness(0.006)  
    mesh.AddVisualShapeFEA(mvisualizeshellB)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Shells FEA test: triangle BST elements')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))  
vis.AddTypicalLights()  


mkl_solver = mkl.ChSolverPardisoMKL()  
mkl_solver.LockSparsityPattern(True)  
sys.SetSolver(mkl_solver)  


timestep = 0.005


sys.Setup()  
sys.Update()  


rec_X = chrono.ChFunctionInterp()
rec_Y = chrono.ChFunctionInterp()


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  

    sys.DoStepDynamics(timestep)