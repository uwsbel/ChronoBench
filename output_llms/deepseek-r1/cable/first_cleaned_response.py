import chrono
import chrono_irrlicht as irr
import chrono.fea as fea


beam_length = 2.0
num_elements = 10
node_spacing = beam_length / num_elements
density = 1000.0  
youngs_modulus = 1e9  
cross_section = 0.01 ** 2  
damping_alpha = 0.02
damping_beta = 0.02


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mesh = fea.ChMesh()
system.Add(mesh)


material = fea.ChMaterialCableANCF(density, youngs_modulus, cross_section)


nodes = []
for i in range(num_elements + 1):
    position = chrono.ChVectorD(i * node_spacing, 0, 0)
    slope = chrono.ChVectorD(1, 0, 0)  
    node = fea.ChNodeCableANCF(position, slope)
    nodes.append(node)
    mesh.AddNode(node)


for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetMaterial(material)
    element.SetAlpha(damping_alpha)
    element.SetBeta(damping_beta)
    mesh.AddElement(element)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)
lock = chrono.ChLinkPointFrame()
lock.Initialize(nodes[0], ground)
system.Add(lock)


application = irr.ChIrrApp(system, "ANCF Cable Beam Simulation", 800, 600)
application.AddTypicalCamera(chrono.ChVectorD(1, 0.5, 1))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()


system.SetSolverType(chrono.ChSolver.Type_MINRES)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-10)
system.SetSolverVerbose(False)


time_step = 1e-4
application.SetTimestep(time_step)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStepDynamics(time_step)
    application.EndScene()