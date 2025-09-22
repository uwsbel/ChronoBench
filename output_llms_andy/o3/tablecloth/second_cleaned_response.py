import os, errno
import pychrono.core       as chrono
import pychrono.irrlicht   as chronoirr
import pychrono.fea        as fea
import pychrono.pardisomkl as mkl




out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory", exc)




sys  = chrono.ChSystemSMC()




mesh = fea.ChMesh()
sys.Add(mesh)


density   = 100.0           
E         = 6.0e4           
nu        = 0.0             
thickness = 0.01            

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material    = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)




L_x = 1.0        
L_z = 1.0        

nsections_x = 40
nsections_z = 40




mynodes          = []    
nodePlotA        = None  
nodePlotB        = None  
nodesLoad        = []    
mnodemonitor     = None  
melementmonitor  = None  




for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0.0,
                              iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

        
        if ix == 0 and iz == 0:
            nodePlotA = mnode
        if ix == nsections_x and iz == nsections_z:
            nodePlotB = mnode


nodesLoad.append(nodePlotB)   




for iz in range(nsections_z):
    for ix in range(nsections_x):

        
        def node_at(ix_, iz_):
            return mynodes[iz_ * (nsections_x + 1) + ix_]

        
        melementA = fea.ChElementShellBST()

        n0 = node_at(ix,     iz)
        n1 = node_at(ix + 1, iz)
        n2 = node_at(ix,     iz + 1)

        
        boundary_1 = node_at(ix + 1, iz + 1)                        
        boundary_2 = node_at(ix - 1, iz + 1) if ix > 0       else None
        boundary_3 = node_at(ix + 1, iz - 1) if iz > 0       else None

        melementA.SetNodes(n0, n1, n2, boundary_1, boundary_2, boundary_3)
        melementA.AddLayer(thickness, 0.0, material)
        mesh.AddElement(melementA)

        
        if iz == 0 and ix == 1:
            melementmonitor = melementA

        
        melementB = fea.ChElementShellBST()

        n3 = node_at(ix + 1, iz + 1)
        n4 = node_at(ix,     iz + 1)
        n5 = node_at(ix + 1, iz)

        boundary_1B = node_at(ix,     iz)                   
        boundary_2B = node_at(ix + 2, iz) if ix < nsections_x - 1 else None
        boundary_3B = node_at(ix,     iz + 2) if iz < nsections_z - 1 else None

        melementB.SetNodes(n3, n4, n5, boundary_1B, boundary_2B, boundary_3B)
        melementB.AddLayer(thickness, 0.0, material)
        mesh.AddElement(melementB)




mid_ix = nsections_x // 2
mid_iz = nsections_z // 2
mnodemonitor = mynodes[mid_iz * (nsections_x + 1) + mid_ix]




for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)




load_force = chrono.ChVector3d(0.0, -100.0, 0.0)     
for n in nodesLoad:
    load = fea.ChLoadXYZnode(n, load_force)
    mesh.AddLoad(load)




ref_X = chrono.ChFunction_Recorder()
ref_Y = chrono.ChFunction_Recorder()




mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)


mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Shells FEA test: triangle BST elements")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 0.3, 1.3),
              chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()




mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)     
sys.SetSolver(mkl_solver)




sys.Setup()
sys.Update()




timestep = 0.005     




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    ref_X.AddPoint(sys.GetChTime(), mnodemonitor.GetPos().x)
    ref_Y.AddPoint(sys.GetChTime(), mnodemonitor.GetPos().y)

    sys.DoStepDynamics(timestep)