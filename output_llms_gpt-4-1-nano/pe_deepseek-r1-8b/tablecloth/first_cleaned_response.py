import os
import math
import numpy as np
from pychrono import chrono
from pychrono.robot import *
from pychrono.irrlicht import *
from pychrono.vehicle import *


sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath('path/to/data')  


material = chrono.ChContactMaterialKirchhoff()
material.SetFriction(0.3)  
material.SetDampingF(0.1)  
material.SetCompliance(0.01)  


grid_size = 0.01  
num_div_x = 50
num_div_y = 50
mesh = fea.ChMesh()


base_section = fea.ChSectionPlatexyz()
base_section.SetLengthX(grid_size * num_div_x)
base_section.SetLengthY(grid_size * num_div_y)
base_section.SetYoungModulus(1e6)  
base_section.SetShearModulus(0.1 * 1e6)  
base_section.SetPoissonRatio(0.3)  


node_positions = []
for i in range(num_div_x + 1):
    for j in range(num_div_y + 1):
        pos = chrono.ChVector3d(
            i * grid_size - grid_size / 2,
            j * grid_size - grid_size / 2,
            0.0  
        )
        node = fea.ChNodeFEAxyzrot(chrono.ChFramed(pos))
        node_positions.append(node)


elements = []
for i in range(num_div_x):
    for j in range(num_div_y):
        
        nodes = [
            node_positions[i * num_div_y + j],
            node_positions[i * num_div_y + j + 1],
            node_positions[(i + 1) * num_div_y + j],
            node_positions[(i + 1) * num_div_y + j + 1]
        ]
        
        element = fea.ChElementShellKirchhoff()
        element.SetMaterial(material)
        element.SetNodes(nodes)
        elements.append(element)


for node in node_positions:
    mesh.AddNode(node)
for element in elements:
    mesh.AddElement(element)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()
vis.AddSkyBox()
cam_pos = chrono.ChVector3d(5, 5, 10)
vis.AddCamera(cam_pos)


sys.SetSolver(chrono.ChSolverPardisoMKL())


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)