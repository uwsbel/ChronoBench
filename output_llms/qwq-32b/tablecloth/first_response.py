import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the simulation system with PardisoMKL solver
system = chrono.ChSystemMyChrono()
system.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Setup visualization
visualiz = chronoirr.ChVisualSystemIrrlicht()
visualiz.SetWindowSize(1024, 768)
visualiz.SetWindowTitle('Tablecloth Folding Simulation')
visualiz.SetSymbolscale(0.01)
visualiz.SetCameraPosition(chrono.ChVectorD(1, 1, 2))
visualiz.SetLightIntensity(0.8)
system.SetVisualSystem(visualiz)

# Create the tablecloth mesh
mesh = chrono.ChMesh()
nx, ny = 20, 20
spacing = 0.05  # meters

# Add nodes in a grid
for i in range(nx):
    for j in range(ny):
        x = i * spacing
        y = j * spacing
        z = 0.5  # initial height above table
        node = chrono.ChNodeFEAxyzD()
        node.SetPos(chrono.ChVectorD(x, y, z))
        mesh.AddNode(node)

# Create shell elements with Kirchhoff material
material = chrono.ChMaterialShellKirchhoff()
material.SetYoungModulus(1e6)  # Pa
material.SetPoissRatio(0.3)
material.SetDensity(800)  # kg/m³

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

# Create FEM actor and add to system
actor = chrono.ChBodyFEMobject()
actor.SetFEMMesh(mesh)
actor.SetBodyFixed(False)  # Allow movement under gravity
system.Add(actor)

# Add collision model to FEM actor
actor.GetCollisionModel().ClearModel()
actor.GetCollisionModel().AddFEMMesh(mesh)
actor.GetCollisionModel().BuildModel()

# Create table as a rigid body
table = chrono.ChBody()
table.SetBodyFixed(True)
table.SetPos(chrono.ChVectorD(0.5, 0.5, 0.05))  # Centered under cloth
table_shape = chrono.ChBox(table.GetFrame(), 1, 1, 0.1)  # 1x1x0.1m box
table.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.5, 0.5, 0.5)))  # Gray color

# Setup collision for table
table.GetCollisionModel().ClearModel()
table.GetCollisionModel().AddBox(0.5, 0.5, 0.05)  # Half-extents
table.GetCollisionModel().BuildModel()
system.Add(table)

# Visualization assets
actor.AddAsset(chrono.ChVisualizationFEMMesh())

# Simulation loop
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