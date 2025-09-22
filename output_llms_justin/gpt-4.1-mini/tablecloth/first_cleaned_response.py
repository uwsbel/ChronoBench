import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr




system = chrono.ChSystemSMC()




cloth_length = 1.0    
cloth_width = 1.0     
num_div_x = 30        
num_div_y = 30        
node_spacing_x = cloth_length / num_div_x
node_spacing_y = cloth_width / num_div_y


mesh = fea.ChMesh()


nodes = []
for i in range(num_div_x + 1):
    for j in range(num_div_y + 1):
        
        x = i * node_spacing_x
        y = j * node_spacing_y
        z = 0.0
        node = fea.ChNodeFEAxyzrot(chrono.ChVectorD(x, y, z), chrono.ChQuaternionD())
        node.SetMass(0.01)  
        mesh.AddNode(node)
        nodes.append(node)


mat = fea.ChMaterialShellKirchhoff()
mat.SetYoungModulus(5e6)   
mat.SetPoissonRatio(0.3)
mat.SetThickness(0.002)    
mat.SetAlphaDamp(0.05)     


def node_index(i, j):
    return i * (num_div_y + 1) + j


for i in range(num_div_x):
    for j in range(num_div_y):
        
        n0 = nodes[node_index(i, j)]
        n1 = nodes[node_index(i + 1, j)]
        n2 = nodes[node_index(i + 1, j + 1)]
        n3 = nodes[node_index(i, j + 1)]

        
        elem1 = fea.ChElementShellReissner4()
        elem1.SetNodes(n0, n1, n2, n3)
        elem1.SetMaterial(mat)
        elem1.SetAlphaDamp(0.05)
        mesh.AddElement(elem1)


for j in range(num_div_y + 1):
    node = nodes[node_index(0, j)]
    node.SetFixed(True)


system.Add(mesh)




system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


solver = chrono.ChSolverPardisoMKL()
solver.LockSparsity(True)
system.SetSolver(solver)



application = irr.ChIrrApp(system, "Tablecloth Folding with Shell Elements", irr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(cloth_length * 0.5, cloth_width * -2.0, 1.5))


application.AssetBindAll()
application.AssetUpdateAll()


visualize_mesh = True
if visualize_mesh:
    
    fea_mesh_shape = fea.ChVisualizationFEAmesh(mesh)
    fea_mesh_shape.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
    fea_mesh_shape.SetWireframe(False)
    fea_mesh_shape.SetColorscaleMinMax(0.0, 1.0)
    mesh.AddVisualShape(fea_mesh_shape)


time_step = 1e-3
application.SetTimestep(time_step)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()