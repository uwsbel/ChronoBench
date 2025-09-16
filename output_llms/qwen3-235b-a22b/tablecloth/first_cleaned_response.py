import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


mesh = fea.ChMesh()


material = fea.ChMaterialShellKirchhoff()
material.SetYoungModulus(2e5)  
material.SetPoissonRatio(0.3)
material.SetDensity(1000)      
material.SetThickness(0.001)   


nx, ny = 20, 20                
lx, ly = 1.0, 1.0              
dx, dy = lx / nx, ly / ny      


nodes = []
for i in range(nx + 1):
    row = []
    for j in range(ny + 1):
        x = i * dx
        z = j * dy
        node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, 0, z))
        
        if i == 0 or i == nx:
            node.SetFixed(True)
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)


for i in range(nx):
    for j in range(ny):
        
        n1 = nodes[i][j]
        n2 = nodes[i+1][j]
        n3 = nodes[i+1][j+1]
        n4 = nodes[i][j+1]
        
        
        el = fea.ChElementShellANCF_3423()
        el.SetNodes(n1, n2, n3, n4)
        
        el.AddLayer(material, 0.001, chrono.ChVectorD(0, 1, 0))  
        mesh.AddElement(el)


system.Add(mesh)


system.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
system.SetSolverMaxIterations(100)


vis = irr.ChIrrApp(system, 'Tablecloth Folding Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalSky()
vis.AddTypicalLogo()
vis.AddTypicalCamera(irr.vector3df(0, 3, -4))  
vis.AddTypicalLights()
vis.AssetBindAll()
vis.AssetUpdateAll()


while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)