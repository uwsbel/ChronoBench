import pychrono as chrono
from pychrono import visualization
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))


application = visualization.ChIrrApp(system, "Beam Simulation", visualization.dimension2d_i(800, 600))
application.AddCamera(chrono.ChVectorD(0, 2, 5))
application.SetTimestep(1 / 60)


nodes = []
for i in range(5):
    node = chrono.ChBody()
    node.SetPos(chrono.ChVectorD(i, 0, 0))
    node.SetMass(0)
    node.SetBodyFixed(False)
    system.Add(node)
    nodes.append(node)


section = chrono.ChBeamSectionEuler()
section.SetYoungModulus(2e9)  
section.SetPoissonRatio(0.3)
section.SetDensity(7850)      
section.SetCrossSection(chrono.ChVectorD(0.05, 0.05))  


for i in range(4):
    beam = chrono.ChBeamElement()
    beam.SetSection(section)
    beam.SetNodes(nodes[i], nodes[i + 1])
    system.Add(beam)


for node in nodes:
    sphere = visualization.ChVisualShapeSphere(0.02)
    sphere.SetMaterialTexture(0, "textures/white.png")
    node.AddVisualShape(sphere)


application.AddLightPoint(chrono.ChVectorD(0, 2, 5), 10)


while application.GetApplication().IsRunning():
    application.BeginScene()
    application.DrawAll()
    
    
    driver = application.GetDevice().getVideoDriver()
    for i in range(4):
        node1 = nodes[i]
        node2 = nodes[i + 1]
        pos1 = node1.GetPos()
        pos2 = node2.GetPos()
        driver.draw3DLine(pos1, pos2, chrono.SColor(255, 255, 0, 0))
    
    application.EndScene()
    application.DoStep()

application.Run()