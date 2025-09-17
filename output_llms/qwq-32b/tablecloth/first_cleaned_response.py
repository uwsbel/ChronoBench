import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemMyChrono()
system.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


visualiz = chronoirr.ChVisualSystemIrrlicht()
visualiz.SetWindowSize(1024, 768)
visualiz.SetWindowTitle('Tablecloth Folding Simulation')
visualiz.SetSymbolscale(0.01)
visualiz.SetCameraPosition(chrono.ChVectorD(1, 1, 2))
visualiz.SetLightIntensity(0.8)
system.SetVisualSystem(visualiz)


mesh = chrono.ChMesh()
nx, ny = 20, 20
spacing = 0.05  


for i in range(nx):
    for j in range(ny):
        x = i * spacing
        y = j * spacing
        z = 0.5  
        node = chrono.ChNodeFEAxyzD()
        node.SetPos(chrono.ChVectorD(x, y, z))
        mesh.AddNode(node)


material = chrono.ChMaterialShellKirchhoff()
material.SetYoungModulus(1e6)  
material.SetPoissRatio(0.3)
material.SetDensity(800)  

for i in range(nx - 1):
    for j in range(ny - 1):
        n0 = i * ny + j
        n1 = (i + 1) * ny + j
        n2 = (i + 1) * ny + (j + 1)
        n3 = i * ny + (j + 1)
        elem = chrono.ChElementShellQuadrilateral()
        elem.SetNodes(mesh.GetNode(n0), mesh.GetNode(n1), mesh.GetNode(n2), mesh.GetNode(n3))
        elem.SetMaterial(material)
        mesh.AddElement(elem)


actor = chrono.ChBodyFEMobject()
actor.SetFEMMesh(mesh)
actor.SetBodyFixed(False)  
system.Add(actor)


actor.GetCollisionModel().ClearModel()
actor.GetCollisionModel().AddFEMMesh(mesh)
actor.GetCollisionModel().BuildModel()


table = chrono.ChBody()
table.SetBodyFixed(True)
table.SetPos(chrono.ChVectorD(0.5, 0.5, 0.05))  
table_shape = chrono.ChBox(table.GetFrame(), 1, 1, 0.1)  
table.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.5, 0.5, 0.5)))  


table.GetCollisionModel().ClearModel()
table.GetCollisionModel().AddBox(0.5, 0.5, 0.05)  
table.GetCollisionModel().BuildModel()
system.Add(table)


actor.AddAsset(chrono.ChVisualizationFEMMesh())


visualiz.Initialize()
visualiz.AddLogo()
visualiz.AddSkySphere()
visualiz.AddLight(chrono.ChVectorD(3, 3, 5), chrono.ChColor(1, 1, 1), 200)
visualiz.AddCamera(chrono.ChVectorD(1, 1, 2))

while system.GetChTime() < 10:
    system.DoStepDynamics(0.005)
    visualiz.BeginScene()
    visualiz.DrawAll()
    visualiz.EndScene()
    if chronoirr.irr_driver is not None and chronoirr.irr_driver.IsKeyDown(chronoirr.keyKEY_ESCAPE):
        break