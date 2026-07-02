import pychrono as chrono
from pychrono import irrlicht
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))


num_elements = 10
num_nodes = num_elements + 1
nodes = []
for i in range(num_nodes):
    node = chrono.ChNodeANCF()
    node.SetPos(chrono.ChVectorD(i, 0, 0))
    nodes.append(node)
    system.Add(node)


elements = []
for i in range(num_elements):
    
    element = chrono.ChBeamANCF(nodes[i], nodes[i+1])
    element.SetCrossSectionArea(0.001)  
    element.SetYoungModulus(1e9)       
    element.SetDensity(1000)           
    elements.append(element)
    system.Add(element)



fixed_body = chrono.ChBody()
fixed_body.SetPos(chrono.ChVectorD(0, 0, 0))
fixed_body.SetFixed(True)
system.Add(fixed_body)


constraint = chrono.ChLinkPoint()
constraint.SetPointA(fixed_body.GetPos())
constraint.SetPointB(nodes[0].GetPos())
system.Add(constraint)


app = irrlicht.ChIrrApp(system, "ANCF Beam Simulation", irrlicht.dimension2d_i(800, 600))
app.AddCamera(chrono.ChVectorD(0, 5, 10))
app.SetTimestep(1e-3)


for node in nodes:
    shape = chrono.ChVisualShapeSphere(0.05)
    shape.SetMaterial(chrono.ChMaterialSurfaceMaterno())
    node.AddVisualShape(shape)


for i in range(num_elements):
    line = chrono.ChVisualShapeLine()
    line.SetLine(nodes[i].GetPos(), nodes[i+1].GetPos())
    line.SetColor(chrono.ChColor(1, 0, 0))
    elements[i].AddVisualShape(line)


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    system.DoStepDynamics(app.GetTimestep())