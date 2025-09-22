import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.mkl as mkl


system = chrono.ChSystemNSC()


solver = mkl.ChSolverPardisoMKL()
system.SetSolver(solver)


mesh = chrono.ChMesh()


Lx = 1.0  
Ly = 1.0  
Nx = 10   
Ny = 10   
thickness = 0.001  
density = 1000     
E = 2.1e9          
nu = 0.3           


nodes = []
for i in range(Nx + 1):
    row = []
    for j in range(Ny + 1):
        x = Lx * i / Nx
        y = Ly * j / Ny
        z = 0.0
        node = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(x, y, z)))
        
        if (i == 0 and j == 0) or (i == Nx and j == 0) or \
           (i == 0 and j == Ny) or (i == Nx and j == Ny):
            node.SetFixed(True)
        row.append(node)
        mesh.AddNode(node)
    nodes.append(row)


for i in range(Nx):
    for j in range(Ny):
        n1 = nodes[i][j]
        n2 = nodes[i+1][j]
        n3 = nodes[i+1][j+1]
        n4 = nodes[i][j+1]
        element = chrono.ChElementShellKirchhoff()
        element.SetNodes(n1, n2, n3, n4)
        material = chrono.ChMaterialShellKirchhoff(E, nu, thickness, density)
        element.SetMaterial(material)
        mesh.AddElement(element)


system.Add(mesh)


visual_asset = chrono.ChVisualizationFEAmesh(mesh)
visual_asset.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_PLOT_SURFACE)
visual_asset.SetWireframe(True)
mesh.AddAsset(visual_asset)
mesh.AddAsset(chrono.ChColorAsset(0.8, 0.2, 0.2))  


application = irr.ChIrrApp(system, 'Tablecloth Folding Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalCamera(irr.vector3df(0, 0, 2), irr.vector3df(0, 0, 0))
application.AddTypicalLights()

application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()