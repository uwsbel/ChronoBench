import numpy as np
from chrono import chrono_types
from chrono import ChronoEngine
from chrono import Irrlicht
from chrono import PyChrono


chrono_types.ChTimeStep.SetDefault(0.01)


system = chrono.ChSystem()


application = Irrlicht.ChIrrApp(system, "ANCF Beam Simulation")


application.SetVerbose(True)
application.SetFrameRate(60)
application.SetCollisionSystemType(chrono.ChCollisionSystem.Type.BULLET)


ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
system.AddBody(ground)


hinge = chrono.ChLinkLockHinge()
hinge.Init(ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)), 
           chrono.ChBody(), chrono.ChFrameD(chrono.ChVectorD(5, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(hinge)


num_elements = 10
beam_length = 10.0
beam_mass = 1.0
beam_E = 2.0e6
beam_rho = 2700
beam_A = 0.01


for i in range(num_elements):
    node1 = chrono.ChNodeFEA(chrono.ChVectorD(i * beam_length / num_elements, 0, 0), 
                             chrono.ChQuaternionD(1, 0, 0, 0), 
                             beam_mass / num_elements, 
                             beam_E, 
                             beam_rho, 
                             beam_A)
    if i == 0:
        node1.SetFixed(True)
    system.AddNode(node1)

    if i > 0:
        element = chrono.ChElementBeamANCF(beam_E, beam_rho, beam_A, 0.0)
        element.SetNodes(node0, node1)
        system.AddElement(element)


system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))


floor = chrono.ChBody()
floor.SetPos(chrono.ChVectorD(0, 0, -10))
floor_shape = chrono.ChBoxShape(chrono.ChVectorD(0, 0, -10), chrono.ChVectorD(20, 20, 0.1))
floor.AddShape(floor_shape)
floor.SetBodyFixed(True)
system.AddBody(floor)


for i in range(num_elements):
    node = system.GetNode(i)
    node_shape = chrono.ChSphereShape(chrono.ChVectorD(0, 0, 0), 0.05)
    node.AddShape(node_shape)


application.Run()