import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl 
import errno
import os




out_dir = "FEA_SHELLS_BST_OUTPUT" 
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")
    else:
        
        pass



sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)


density = 100
E = 6e4
nu = 0.0
thickness = 0.01


melasticity = fea.ChElasticityKirchhoffIsotropic(E, nu) 
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




node_idx_plotA_iz = nsections_z // 2
node_idx_plotA_ix = nsections_x // 2
nodePlotA = mynodes[node_idx_plotA_iz * (nsections_x + 1) + node_idx_plotA_ix]

node_idx_plotB_iz = nsections_z
node_idx_plotB_ix = nsections_x
nodePlotB = mynodes[node_idx_plotB_iz * (nsections_x + 1) + node_idx_plotB_ix]


nodesLoad = []




if len(mynodes) > nsections_x // 2 : 
    nodesLoad.append(mynodes[0 * (nsections_x + 1) + nsections_x // 4])
    nodesLoad.append(mynodes[0 * (nsections_x + 1) + nsections_x // 2])
    nodesLoad.append(mynodes[0 * (nsections_x + 1) + 3 * nsections_x // 4])



ref_X = chrono.ChFunction_Recorder()
ref_X.AddPoint(0, 0)
ref_X.AddPoint(1, 0.1)
ref_X.AddPoint(2, 0.2)

ref_Y = chrono.ChFunction_Recorder()
ref_Y.AddPoint(0, 0)
ref_Y.AddPoint(1, 0.05)
ref_Y.AddPoint(2, 0.15)


load_force = chrono.ChVector3d(0, -10, 0) 


mnodemonitor_iz = nsections_z // 2
mnodemonitor_ix = nsections_x // 2 + 1 
mnodemonitor = mynodes[mnodemonitor_iz * (nsections_x + 1) + mnodemonitor_ix]


melementmonitor = None






fix_dim_z = 30
fix_dim_x = 30
for j in range(min(fix_dim_z, nsections_z + 1)): 
    for k in range(min(fix_dim_x, nsections_x + 1)): 
        node_to_fix = mynodes[j * (nsections_x + 1) + k]
        node_to_fix.SetFixed(True)







for iz in range(nsections_z):
    for ix in range(nsections_x):
        
        n0_idx = iz * (nsections_x + 1) + ix
        n1_idx = iz * (nsections_x + 1) + ix + 1
        n2_idx = (iz + 1) * (nsections_x + 1) + ix
        n3_idx = (iz + 1) * (nsections_x + 1) + ix + 1

        node0 = mynodes[n0_idx] 
        node1 = mynodes[n1_idx] 
        node2 = mynodes[n2_idx] 
        node3 = mynodes[n3_idx] 

        
        
        
        
        
        adj_A1 = node3
        adj_A2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else None
        adj_A3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else None
        
        melementA = fea.ChElementShellBST()
        melementA.SetNodes(node0, node1, node2, adj_A1, adj_A2, adj_A3)
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        
        if (iz == 0 and ix == 1):
            melementmonitor = melementA
        

        
        
        
        
        
        adj_B1 = node0
        adj_B2 = mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None
        adj_B3 = mynodes[(iz + 2) * (nsections_x + 1) + ix + 1] if iz < nsections_z - 1 else None 
                                                                                                    
                                                                                                    
                                                                                                    
                                                                                                    
                                                                                                    
                                                                                                    
                                                                                                    
        adj_B3_original_logic = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None


        melementB = fea.ChElementShellBST()
        melementB.SetNodes(node3, node2, node1, adj_B1, adj_B2, adj_B3_original_logic) 
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)




mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE) 
mvisualizeshellA.SetShellResolution(2) 




mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)

mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mesh.AddVisualShapeFEA(mvisualizeshellB)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements (Modified)')
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