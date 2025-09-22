import chrono as ch
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


system = ch.ChSystemEulerImplicit()


mesh = ch.ChMesh()


node1 = ch.ChNodeFEA(0, 0, 0)
node2 = ch.ChNodeFEA(1, 0, 0)
node3 = ch.ChNodeFEA(0, 1, 0)
node4 = ch.ChNodeFEA(0, 0, 1)


mesh.AddNode(node1)
mesh.AddNode(node2)
mesh.AddNode(node3)
mesh.AddNode(node4)


elem1 = ch.ChElementBeamTimoshenko(node1, node2, 0.1, 0.01, 0.01, 0.1)
elem2 = ch.ChElementBeamTimoshenko(node1, node3, 0.1, 0.01, 0.01, 0.1)
elem3 = ch.ChElementBeamTimoshenko(node1, node4, 0.1, 0.01, 0.01, 0.1)


mesh.AddElement(elem1)
mesh.AddElement(elem2)
mesh.AddElement(elem3)


material = ch.ChMaterialShellANCF()
material.SetDensity(1000)
material.SetYoungModulus(1e6)
material.SetPoissonRatio(0.3)


elem1.SetMaterial(material)
elem2.SetMaterial(material)
elem3.SetMaterial(material)


system.Add(mesh)


app = irr.ChIrrApp(system, 'Beam Finite Elements Simulation', irr.Dim2D)


app.SetCameraPosition(ch.ChVector3d(0, -5, 0))


app.EnableVisualization(True)


while app.Run():
    system.DoStepDynamics(0.01)