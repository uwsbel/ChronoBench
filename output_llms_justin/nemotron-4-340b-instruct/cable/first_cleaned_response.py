import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.ChSystemSMC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('ANCF Cable Beam Simulation')
vis.Initialize()


material = chrono.ChMaterialSurfaceSMC()
material.SetFriction(0.3)


mesh = chrono.ChMesh()


num_elements = 10  
element_length = 0.5  
cable_radius = 0.01  


for i in range(num_elements):
    nodeA = chrono.ChNodeFEAxyz(chrono.ChFrameD(chrono.ChVectorD(i * element_length, 0, 0)))
    nodeB = chrono.ChNodeFEAxyz(chrono.ChFrameD(chrono.ChVectorD((i + 1) * element_length, 0, 0)))

    element = chrono.ChElementCableANCF(nodeA, nodeB, cable_radius, material)
    mesh.AddElement(element)


ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True, chrono.ChFrameD(chrono.ChVectorD(0, 0, -0.05)))
chrono.GetSystem().Add(ground)


hinge_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hinge_body.SetBodyFixed(True)
chrono.GetSystem().Add(hinge_body)


hinge_joint = chrono.ChLinkLockRevolute()
hinge_joint.Initialize(hinge_body, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
chrono.GetSystem().Add(hinge_joint)


chrono.GetSystem().Add(mesh)


chrono.GetSystem().Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


for node in mesh.GetNodes():
    node.SetPos(chrono.ChVectorD(node.GetPos().x(), 0, 0))


for element in mesh.GetElements():
    element.SetVisualizationType(chrono.ChVisualizationFEA.E_BEAM_ELEM)
    element.SetVisualizationColor(chrono.ChColor(0.8, 0.8, 0.8))


while vis.Run():
    chrono.GetSystem().DoStepDynamics(0.01)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    for node in mesh.GetNodes():
        print(f'Node position: {node.GetPos()}')