import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, irrrlicht, vehicle
from pychrono.robot import *
from pychrono.irrlicht import *
from pychrono.vehicle import *


sys = chrono.ChSystemNSC()


chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


num_elements = 10  
length = 10.0  
element_length = length / num_elements  
young_modulus = 0.01e9  
shear_modulus = 0.3 * young_modulus  
rayleigh_damping = 0.000  


beam = fea.ChBeam()
beam.SetMaterial(fea.ChMaterial(fea.MaterialType_ANCF, young_modulus, shear_modulus, rayleigh_damping))


nodes = []
for i in range(num_elements + 1):
    pos = chrono.ChVector3d()
    rot = chrono.ChQuatf()
    if i == 0:
        pos.Set(0, 0, 0)
        rot.Set(1, 0, 0, 1)
    elif i == num_elements:
        pos.Set(element_length * num_elements, 0, 0)
        rot.Set(1, 0, 0, 1)
    else:
        pos.Set((i - 1) * element_length, 0, 0)
        rot.Set(1, 0, 0, 1)
    nodes.append(fea.ChNodeFEAxyzrot(chrono.ChFramed(pos, rot)))


for i in range(num_elements):
    node1 = nodes[i]
    node2 = nodes[i + 1]
    elem = fea.ChElementBeamANCF()
    elem.SetNodes(node1, node2)
    elem.SetSection(beam)
    beam.AddElement(elem)


sys.Add(beam)


beam.GetNode(0).SetFixed(True)


gravity = chrono.ChVector3d(0, -9.81, 0)
beam.GetNode(num_elements).AddForceAndTorque(gravity)


contact_material = chrono.ChContactMaterialNSC()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.1)
contact_material.SetCompliance(0.01)
sys.SetContactMaterial(contact_material)


collision_model = sys.GetCollisionModel()
collision_model.SetDefaultSuggestedEnvelope(0.01)
collision_model.SetDefaultSuggestedMargin(0.005)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()


node_shape = chrono.ChVisualShapeSphere(0.05)
ground_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(50, 50, 5))
sys.GetNode(0).AddVisualShape(node_shape)
sys.GetNode(num_elements).AddVisualShape(node_shape)
ground_node = fea.ChNodeStatic()
ground_node.SetPos(chrono.ChVector3d(0, -5, 0))
ground_node.AddVisualShape(ground_shape)
sys.Add(ground_node)


time_step = 0.001
simulation_speed = 1.0
key_pressed = False

while True:
    if vis.Run(key_pressed):
        key_pressed = False
        sys.DoStepDynamics(time_step)
        sys.UpdateContactContainer()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    else:
        time_step = max(1e-8, min(0.001 / simulation_speed, 0.001))
        if key_pressed:
            simulation_speed = 1.0
        else:
            simulation_speed = 10.0
    if chr(vis.GetKey()) == ' ':
        simulation_speed = 0.1
    elif chr(vis.GetKey()) == 'p':
        simulation_speed = 10.0
    elif chr(vis.GetKey()) == 'q':
        break

vis.Destroy()
sys.Destroy()