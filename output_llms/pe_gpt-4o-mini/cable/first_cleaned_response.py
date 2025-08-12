import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


sys = chrono.ChSystemSMC()


sys.Set_G_acc(chrono.ChVector(0, -9.81, 0))


mesh = fea.ChMesh()


length = 5.0  
num_elements = 10  
element_length = length / num_elements  
diameter = 0.1  


for i in range(num_elements + 1):
    
    node = fea.ChNodeFEAxyzrot(chrono.ChFrame(chrono.ChVector3d(i * element_length, 0, 0), chrono.QUNIT))
    mesh.AddNode(node)


for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(mesh.GetNode(i), mesh.GetNode(i + 1))
    element.SetDiameter(diameter)
    element.SetDensity(1000)  
    mesh.AddElement(element)


sys.Add(mesh)


first_node = mesh.GetNode(0)
first_node.SetFixed(True)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Cable Beam Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 3, 10))
vis.AddTypicalLights()


time_step = 0.01
while vis.Run():
    sys.DoStepDynamics(time_step)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    for i in range(num_elements + 1):
        node_pos = mesh.GetNode(i).GetPos()
        print(f'Node {i} Position: {node_pos}')