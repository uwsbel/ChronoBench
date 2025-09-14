import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht


chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_ROOT'], 'data/'))
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


mesh = fea.ChMesh()
beam_length = 2.0
n_nodes = 10
section = fea.ChBeamSectionEulerAdvanced()


rho = 7850  
E = 2e11    
nu = 0.3    
section.SetDensity(rho)
section.SetYoungModulus(E)
section.SetBeamRectangular(0.02, 0.02)  


nodes = []
for i in range(n_nodes):
    pos = chrono.ChVectorD(0, i * beam_length/(n_nodes-1), 0)
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(pos))
    node.SetMass(0)
    mesh.AddNode(node)
    nodes.append(node)

for i in range(n_nodes-1):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(section)
    mesh.AddElement(element)


constraint = fea.ChLinkPointFrame()
constraint.Initialize(nodes[0], nodes[0].Frame().GetCoord())
mesh.AddConstraint(constraint)


motor_function = chrono.ChFunction_Ramp(0, 0.001)  
motor = fea.ChLoadMotorLinearPosition(nodes[-1].GetBody())
motor.SetMotionLaw(motor_function)
motor.SetMotorFunction(motor_function)

load_container = chrono.ChLoadContainer()
load_container.Add(motor)
system.Add(load_container)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0.3, 1.0, 0.3), chrono.ChVectorD(0, 0.5, 0))
vis.AddTypicalLights()


fea_vis = fea.ChVisualizationFEAmesh(mesh)
fea_vis.SetFEMdataType(fea.FEADataType_ELEM_BEAM_MZ)
fea_vis.SetColorscaleMinMax(-100, 100)
fea_vis.SetSmoothFaces(True)
fea_vis.SetWireframe(False)
mesh.AddVisualizationFEA(fea_vis)

system.Add(mesh)


solver = chrono.ChSolverMINRES()
system.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)

system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
integrator = system.GetTimestepper().AsTimestepperHHT()
integrator.SetAlpha(-0.2)
integrator.SetStepControl(True)
integrator.SetVerbose(False)


dt = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(dt)