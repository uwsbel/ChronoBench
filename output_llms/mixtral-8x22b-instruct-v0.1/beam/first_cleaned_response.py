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
beam_inertia = np.pi * beam_radius**4 / 4
beam_area = np.pi * beam_radius**2


nodes = [chrono.ChNode(my_system) for _ in range(num_beams + 1)]


beams = [chrono.ChElementBeamEuler() for _ in range(num_beams)]

for i, beam in enumerate(beams):
    beam.SetNodes(nodes[i], nodes[i + 1])
    beam.SetMassPerLength(beam_density * beam_area)
    beam.SetInertiaPerLength(beam_density * beam_inertia)
    beam.SetYoungModulus(beam_young_modulus)
    beam.SetRestLength(beam_length)



for i, node in enumerate(nodes):
    node.SetPos(chrono.ChVectorD(i * beam_length, 0, 0))
    node.SetMass(node_mass)


my_system.AddConstraint_Fixed(nodes[0])


force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE)
force.SetDir(chrono.ChVectorD(0, -1, 0))
force.SetMagnitude(100)
nodes[-1].AddForce(force)


myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono Beam Finite Elements', chronoirr.dimension2du(800, 600))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo()
myapplication.AddTypicalCamera(chronoirr.vector3df(0, 5, -10))
myapplication.AddLightWithShadow(chronoirr.vector3df(5, 5, -5), chronoirr.vector3df(0, -1, 0), 10, 5, 50, 32, 25, 25)

myapplication.AddShadowAll()
myapplication.SetSymbolscale(0.005)

myapplication.AssetBindAll()
myapplication.AssetUpdateAll()

myapplication.Run()