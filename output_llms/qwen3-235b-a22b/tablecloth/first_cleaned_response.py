import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht
import pychrono.mkl as mkl


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


length = 1.0  
width = 1.0   
num_nodes_x = 10
num_nodes_z = 10
dx = length / (num_nodes_x - 1)
dz = width / (num_nodes_z - 1)


mesh = fea.ChMesh()


nodes = []
for i in range(num_nodes_x):
    row = []
    for j in range(num_nodes_z):
        x = i * dx
        z = j * dz
        pos = chrono.ChVectorD(x, 0, z)
        rot = chrono.ChQuaternionD(1, 0, 0, 0)  
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(pos, rot))
        
        if j == 0:
            node.SetFixed(True)
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)


material = fea.ChMaterialShellKirchhoff()
material.SetYoungModulus(2.1e9)    
material.SetPoissonRatio(0.3)      
material.SetThickness(0.001)       


for i in range(num_nodes_x - 1):
    for j in range(num_nodes_z - 1):
        
        n1 = nodes[i][j]
        n2 = nodes[i+1][j]
        n3 = nodes[i+1][j+1]
        n4 = nodes[i][j+1]
        
        
        element = fea.ChElementShellKirchhoff()
        element.SetNodes(n1, n2, n3, n4)
        element.SetMaterial(material)
        element.SetAlphaDamp(0.0)  
        mesh.AddElement(element)


sys.Add(mesh)


solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 1, -2))  
vis.AddLight(chrono.ChVectorD(1, 2, -1), 2.5)  


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)