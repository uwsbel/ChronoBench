import pychrono as ch
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


system = ch.ChSystemSMC()
system.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
system.GetCollisionSystem().SetDefaultSuggestedEnvelope(0.005)
system.GetCollisionSystem().SetDefaultSuggestedMargin(0.0025)
system.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))


tablecloth = fea.ChMesh()
tablecloth.SetAutomaticGravity(False)
system.Add(tablecloth)


material = fea.ChMaterialShellKirchhoff()
material.SetDensity(7800)
material.SetThickness(0.01)
material.SetYoungModulus(2e11)
material.SetPoissonRatio(0.3)
material.SetShearModulus(7.69230769e10)


mesh_size = 0.1
num_nodes_x = 20
num_nodes_y = 10


nodes = []
for j in range(num_nodes_y):
    for i in range(num_nodes_x):
        node = fea.ChNodeFEAxyzrot(ch.ChVector3d(i * mesh_size, j * mesh_size, 0))
        node.SetMass(0)
        node.AddForce(ch.ChVector3d(0, 0, 0))
        node.SetFixed(False)
        tablecloth.AddNode(node)
        nodes.append(node)


for j in range(num_nodes_y - 1):
    for i in range(num_nodes_x - 1):
        element = fea.ChElementShellKirchhoff()
        element.SetNodes(
            nodes[j * num_nodes_x + i],
            nodes[j * num_nodes_x + i + 1],
            nodes[(j + 1) * num_nodes_x + i + 1],
            nodes[(j + 1) * num_nodes_x + i]
        )
        element.SetMaterial(material)
        element.SetAlphaDamp(0.0)
        tablecloth.AddElement(element)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth simulation')
vis.Initialize()
vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(ch.ChVector3d(1, 1.5, 1.5), ch.ChVector3d(1, 1, 0))
vis.AddTypicalLights()


system.SetSolver(ch.ChSolver.Type_PARDISO_MKL)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)