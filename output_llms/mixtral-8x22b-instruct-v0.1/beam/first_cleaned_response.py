import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/")  
chrono.InitChrono()


my_system = chrono.ChSystemNSC()


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)


num_beams = 10
node_mass = 1.0
beam_length = 1.0
beam_radius = 0.05
beam_density = 7850.0
beam_E = 210e9  
beam_G = 80e9  
beam_I = np.pi * (beam_radius ** 4) / 4  
beam_A = np.pi * (beam_radius ** 2)  


nodes = []
for i in range(num_beams + 1):
    node = chrono.ChNodeNSC(my_system)
    node.SetMass(node_mass)
    nodes.append(node)


beams = []
for i in range(num_beams):
    beam = chrono.ChBeamSectionNSC()
    beam.SetDensity(beam_density)
    beam.SetYoungModulus(beam_E)
    beam.SetShearModulus(beam_G)
    beam.SetMomentOfInertia(beam_I)
    beam.SetArea(beam_A)

    beam_shape = chrono.ChBeamSectionAdvanced()
    beam_shape.SetRadius(beam_radius)
    beam.SetShape(beam_shape)

    beam_element = chrono.ChElementBeamEuler()
    beam_element.SetBeamSection(beam)

    beams.append(beam_element)


for i in range(num_beams):
    my_system.AddBeam(beams[i], False, nodes[i], nodes[i + 1])



for i in range(num_beams + 1):
    nodes[i].SetPos(chrono.ChVectorD(i * beam_length, 0, 0))


force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE)
force.SetDirection(chrono.ChVectorD(0, -10, 0))
nodes[-1].AddForce(force)


application = chronoirr.ChIrrApp(my_system, "PyChrono Beam Finite Elements Simulation", chronoirr.Dim3d(800, 600))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(10, 5, -10), chronoirr.vector3df(0, 0, 0))
application.AddLightWithShadow(chronoirr.vector3df(10, 10, -10), chronoirr.vector3df(0, 0, 0), 30, 10, 20, 128)


for node in nodes:
    node.SetBodyFixed(False)
    node.SetIdentifier(-1)
    node.SetCollide(True)
    node.SetBodyKinematic()

for beam in beams:
    beam.SetColor(chrono.ChColor(0.8, 0.2, 0.2))


application.Run()