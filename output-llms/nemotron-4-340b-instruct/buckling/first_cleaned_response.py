import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


chrono.ChSystemSMC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, -2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


beam_material = fea.ChMaterialShellANCF(1000, 0.3, 0.001)


beam_mesh = fea.ChMesh()


beam_length = 10
beam_width = 0.1
beam_height = 0.1
beam_nodes = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
beam_nodes.SetMass(0)
beam_mesh.AddNode(beam_nodes)

for i in range(1, 11):
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * beam_length / 10, 0, 0)))
    node.SetMass(0)
    beam_mesh.AddNode(node)

for i in range(10):
    element = fea.ChElementShellANCF()
    element.SetNodes(beam_mesh.GetNode(i), beam_mesh.GetNode(i + 1))
    element.SetMaterial(beam_material)
    beam_mesh.AddElement(element)


chrono.GetSystem().Add(beam_mesh)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam_mesh.GetNode(0), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Ramp(0, 10))
chrono.GetSystem().Add(motor)


constraint = chrono.ChLinkMateFixed()
constraint.Initialize(beam_mesh.GetNode(10), chrono.ChBody())
chrono.GetSystem().Add(constraint)


chrono.GetSystem().Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


timestepper = chrono.ChTimestepperHHT(chrono.GetSystem())
timestepper.SetAlpha(0.0)
timestepper.SetMaxiters(10)
timestepper.SetTolerance(1e-3)
chrono.GetSystem().SetTimestepper(timestepper)


solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-5)
chrono.GetSystem().SetSolver(solver)


while vis.Run():
    chrono.GetSystem().DoStepDynamics(0.01)
    vis.Render()