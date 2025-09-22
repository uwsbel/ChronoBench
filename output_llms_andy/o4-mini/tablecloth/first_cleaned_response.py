import chrono
import chrono_irrlicht
from math import sin, cos, pi




system = chrono.ChSystem()  
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


system.SetSolverType(chrono.ChSolver.PARDISO_MKL)
solver = system.GetSolver()
solver.SetVerbose(False)

solver.LockSparsityPattern(True)




mesh = chrono.ChMesh()


cloth_length_x = 1.0   
cloth_length_y = 1.0   
num_nodes_x   = 21
num_nodes_y   = 21
dx = cloth_length_x / (num_nodes_x - 1)
dy = cloth_length_y / (num_nodes_y - 1)


thickness = 0.005
rho       = 500.0          
E         = 2e7            
nu        = 0.3            
mat_shell = chrono.ChMaterialShellKirchhoff(rho, E, nu, thickness)


nodes = []
for j in range(num_nodes_y):
    y = j * dy
    for i in range(num_nodes_x):
        x = i * dx
        
        node = chrono.ChNodeFEAxyzD(chrono.ChVectorD(x, y, 0),
                                    chrono.ChVectorD(0, 0, 1))
        
        if j == 0:
            node.SetFixed(True)
        mesh.AddNode(node)
        nodes.append(node)


for j in range(num_nodes_y - 1):
    for i in range(num_nodes_x - 1):
        
        n0 = j   * num_nodes_x + i
        n1 = j   * num_nodes_x + (i+1)
        n2 = (j+1)* num_nodes_x + (i+1)
        n3 = (j+1)* num_nodes_x + i

        elm = chrono.ChElementShellANCF()
        elm.SetNodes(nodes[n0], nodes[n1], nodes[n2], nodes[n3])
        elm.SetMaterial(mat_shell)
        elm.SetAlphaDamp(0.2)    
        mesh.AddElement(elm)


system.Add(mesh)


mesh_asset = chrono.ChVisualizationFEAmesh(mesh)
mesh_asset.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED)
mesh_asset.SetColorscaleMinMax(0.0, 2.0)
mesh_asset.SetSmoothFaces(True)
mesh_asset.SetWireframe(True)
mesh.AddAsset(mesh_asset)




app = chrono_irrlicht.ChIrrApp(system, "Tablecloth Folding Demo",
                                chrono_irrlicht.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(1.2, 0.5, 1.2),
                     chrono.ChVectorD(0.5, 0.5, 0.0))

app.AssetBindAll()
app.AssetUpdateAll()




app.SetTimestep(1e-3)
while app.Run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    system.DoStepDynamics(1e-3)