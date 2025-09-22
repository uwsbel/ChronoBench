import pychrono as chrono
import pychrono.fem as fem
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()


mesh = fem.ChMesh()
sys.Add(mesh)


Lx = 1.0  
Ly = 1.0  
Nx = 10   
Ny = 10   
dx = Lx / (Nx - 1)
dy = Ly / (Ny - 1)


nodes = []
for i in range(Nx):
    row = []
    for j in range(Ny):
        x = i * dx
        y = j * dy
        z = 0.0
        node = fem.ChNodeFEAxyz(chrono.ChVectorD(x, y, z))
        mesh.AddNode(node)
        row.append(node)
        
        if ((i == 0 and j == 0) or 
            (i == Nx-1 and j == 0) or 
            (i == Nx-1 and j == Ny-1) or 
            (i == 0 and j == Ny-1)):
            node.SetFixed(True)
    nodes.append(row)


material = fem.ChMaterialShellKirchhoff(
    E=1e6,        
    nu=0.3,       
    density=1000, 
    thickness=0.001 
)


for i in range(Nx - 1):
    for j in range(Ny - 1):
        element = fem.ChElementShellKirchhoff()
        element.SetNodes(
            nodes[i][j],
            nodes[i+1][j],
            nodes[i+1][j+1],
            nodes[i][j+1]
        )
        element.SetMaterial(material)
        mesh.AddElement(element)


sys.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
sys.SetSolverMaxIterations(100)


vis = irr.ChIrrApp(sys, 'Tablecloth Folding Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0))
vis.AssetBindAll()
vis.AssetUpdateAll()


while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()