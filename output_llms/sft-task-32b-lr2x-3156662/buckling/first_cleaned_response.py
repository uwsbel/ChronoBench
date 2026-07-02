import pychrono as chrono
import numpy as np


system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


nodes = []
for i in range(11):
    pos = chrono.ChVector3d(i * 0.1, 0, 0)
    node = chrono.ChNodeFEAxyzrot(pos, chrono.ChQuaterniond(1, 0, 0, 0))
    system.Add(node)
    nodes.append(node)


elements = []
for i in range(10):
    element = chrono.ChElementBeamTimo()
    element.SetNodes(nodes[i], nodes[i+1])
    
    E = 210e9  
    G = 0.78e9  
    area = 0.01  
    Iyy = 1e-6  
    Izz = 1e-6
    element.SetYoungModulus(E)
    element.SetShearModulus(G)
    element.SetSectionArea(area)
    element.SetIyy(Iyy)
    element.SetIzz(Izz)
    element.SetDiameter(0.05)  
    system.Add(element)
    elements.append(element)


fixed_link = chrono.ChLinkNodeFixed(nodes[0])
system.Add(fixed_link)


motor = chrono.ChLinkMotorLinearPosition()
motor.Initialize(nodes[-1], chrono.ChFrameMoving(chrono.ChVector3d(1, 0, 0)), 
                 chrono.ChFramed(chrono.ChVector3d(0, 1, 0), chrono.QUNIT))
motor.SetMotion(chrono.ChFunctionRamp(0, -10))  
system.Add(motor)


for node in nodes:
    sphere = chrono.ChVisualShapeSphere(0.02)
    sphere.SetColor(chrono.ChColor(0.8, 0, 0))
    node.AddVisualShape(sphere)


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Beam Buckling FEA")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, -1, 1), chrono.ChVector3d(0.5, 0, 0))
vis.AddTypicalLights()


system.SetTimestepperType(chrono.ChTimestepper.Type_FEA_IMPLICIT)
solver = system.GetSolver()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-8)
solver.EnableWarmStart(True)


time_end = 2.0
step = 0.001
while system.GetChTime() < time_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step)