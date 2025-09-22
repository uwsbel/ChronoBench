import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


my_system = chrono.ChSystemNSC()



num_beams = 5
node_mass = 1
beam_length = 1
beam_radius = 0.1
beam_density = 7850  
beam_young_modulus = 2.1e11  
beam_poisson_ratio = 0.3  


nodes = []
for i in range(num_beams + 1):
    node = chrono.ChNodeFEAxyzD(my_system)
    node.SetMass(node_mass)
    nodes.append(node)


beams = []
for i in range(num_beams):
    beam = chrono.ChElementBeamEuler(nodes[i], nodes[i + 1], beam_radius, beam_length, beam_density, beam_young_modulus, beam_poisson_ratio)
    beams.append(beam)



for i in range(num_beams + 1):
    nodes[i].SetPos(chrono.ChVectorD(i * beam_length, 0, 0))


myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono Beam Simulation', chronoirr.dimension2du(1024, 768))


myapplication.AddTypicalSky()
myapplication.AddTypicalLogo()
myapplication.AddTypicalCamera(chronoirr.vector3df(0, 10, -20))

while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()