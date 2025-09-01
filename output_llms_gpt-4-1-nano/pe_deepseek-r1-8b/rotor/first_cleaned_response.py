import os
import math
import numpy as np
import pychrono as chrono
from pychrono import fea, robosimian, veh
from pychrono.irrlicht import chronoirr
from pychrono.vehicle import vehicle


sys = chrono.ChSystemSMC()


chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()


beam_L = 4.0  


beam_section = fea.ChBeamSectionIga()
beam_section.SetYoungModulus(0.01e9)  
beam_section.SetShearModulus(0.01e9 * 0.3)  
beam_section.SetRayleighDamping(0.000)  


beam_coordsys = chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI))


nodes = []
nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))
for i in range(1, 11):  
    nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))
nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))


elements = []
for i in range(len(nodes) - 1):
    elem = fea.ChElementBeamIga()
    elem.SetNodes(nodes[i], nodes[i + 1])
    elem.SetSection(beam_section)
    elements.append(elem)
    sys.Add(elem)


mesh = fea.ChMesh()
for node in nodes:
    mesh.AddNode(node)
for elem in elements:
    mesh.AddElement(elem)


flywheel_body = chrono.ChBody()

flywheel_pos = chrono.ChVector3d(beam_L / 2, 0, 0)
flywheel_body.SetPos(flywheel_pos)
flywheel_body.SetMass(1)  
flywheel_body.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))  
flywheel_shape = chrono.ChVisualShapeSphere(0.5)  
flywheel_body.AddVisualShape(flywheel_shape)
sys.Add(flywheel_body)


vis.Add(flywheel_body)


motor_joint = chrono.ChLinkLockRevolute()
motor_coordsys = chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2))  
motor_joint.Initialize(nodes[1], flywheel_body, motor_coordsys)
sys.Add(motor_joint)


motor_speed_func = chrono.ChFunctionSine(0.1, 1.0)  
motor_joint.SetSpeedFunction(motor_speed_func)


beam_visual = chrono.ChVisualShapeBox(chrono.ChVector3d(beam_L, 0.05, 0.05))  
for node in nodes:
    node.SetVisualShape(chrono.ChVisualShapeSphere(0.01))  


for elem in elements:
    elem.SetVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(elem.GetLengthX(), 0.01, 0.01)))  
vis.AddElement(elem)
for node in nodes:
    vis.AddNode(node)


time_step = 0.001


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(time_step)
    vis.EndScene()

import os
import math
import numpy as np
import pychrono as chrono
from pychrono import fea, robosimian, veh
from pychrono.irrlicht import chronoirr
from pychrono.vehicle import vehicle


sys = chrono.ChSystemSMC()


chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()


beam_L = 4.0  


beam_section = fea.ChBeamSectionIga()
beam_section.SetYoungModulus(0.01e9)  
beam_section.SetShearModulus(0.01e9 * 0.3)  
beam_section.SetRayleighDamping(0.000)  


beam_coordsys = chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI))


nodes = []
nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))
for i in range(1, 11):  
    nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))
nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))


elements = []
for i in range(len(nodes) - 1):
    elem = fea.ChElementBeamIga()
    elem.SetNodes(nodes[i], nodes[i + 1])
    elem.SetSection(beam_section)
    elements.append(elem)
    sys.Add(elem)


mesh = fea.ChMesh()
for node in nodes:
    mesh.AddNode(node)
for elem in elements:
    mesh.AddElement(elem)


flywheel_body = chrono.ChBody()

flywheel_pos = chrono.ChVector3d(beam_L / 2, 0, 0)
flywheel_body.SetPos(flywheel_pos)
flywheel_body.SetMass(1)  
flywheel_body.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))  
flywheel_shape = chrono.ChVisualShapeSphere(0.5)  
flywheel_body.AddVisualShape(flywheel_shape)
sys.Add(flywheel_body)


vis.Add(flywheel_body)


motor_joint = chrono.ChLinkLockRevolute()
motor_coordsys = chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2))  
motor_joint.Initialize(nodes[1], flywheel_body, motor_coordsys)
sys.Add(motor_joint)


motor_speed_func = chrono.ChFunctionSine(0.1, 1.0)  
motor_joint.SetSpeedFunction(motor_speed_func)


beam_visual = chrono.ChVisualShapeBox(chrono.ChVector3d(beam_L, 0.05, 0.05))  
for node in nodes:
    node.SetVisualShape(chrono.ChVisualShapeSphere(0.01))  


for elem in elements:
    elem.SetVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(elem.GetLengthX(), 0.01, 0.01)))  
vis.AddElement(elem)
for node in nodes:
    vis.AddNode(node)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.001)
    vis.EndScene()