import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.analytical_functions as mfunc
import numpy as np


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(mysystem, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)


beam_length = 10
beam_mass = 10
beam_density = beam_mass / beam_length
beam_cross_section_area = 0.1
beam_young_modulus = 2e11
beam_poisson_ratio = 0.3


beam_elements = []
num_elements = 10
for i in range(num_elements):
    beam_elements.append(chrono.ChElementCableANCF(
        mysystem, beam_density, beam_cross_section_area, beam_young_modulus, beam_poisson_ratio,
        chrono.ChVectorD(beam_length * i / num_elements, 0, 0),
        chrono.ChVectorD(beam_length * (i + 1) / num_elements, 0, 0)
    ))


for element in beam_elements:
    mysystem.Add(element)


hinge = chrono.ChLinkHinge(
    mysystem, ground, beam_elements[0].GetNode1(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0)
)
mysystem.Add(hinge)


myapplication = chronoirr.ChVisualSystemIrrlicht()
myapplication.SetWindowSize(800, 600)
myapplication.SetWindowTitle('Beam Simulation')
myapplication.SetTypicalLength(1.0)
myapplication.SetCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


while myapplication.GetDevice().run():
    mysystem.DoStepDynamics(0.01)
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()

    
    for i, element in enumerate(beam_elements):
        node1 = element.GetNode1()
        node2 = element.GetNode2()
        print(f'Node {i+1}: {node1.GetPos().x}, {node1.GetPos().y}, {node1.GetPos().z}')
        print(f'Node {i+2}: {node2.GetPos().x}, {node2.GetPos().y}, {node2.GetPos().z}')


del myapplication