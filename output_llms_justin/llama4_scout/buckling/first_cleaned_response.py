import numpy as np
from chrono import chrono_types
from chrono.fea import *
from chrono.irrlicht import *
from chrono.core import *


chrono_types.InitChrono()


system = ChSystemSMC()


application = ChIrrApp(system, "Beam Buckling Simulation")


system.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0))


beam_length = 10.0
beam_height = 0.1
beam_thickness = 0.01
num_elements = 10

beam_mesh = ChMesh()


for i in range(num_elements + 1):
    x = i / num_elements * beam_length
    node = ChNodeFEA(ChVector3d(x, 0, 0))
    beam_mesh.AddNode(node)


for i in range(num_elements):
    nodeA = beam_mesh.GetNode(i)
    nodeB = beam_mesh.GetNode(i + 1)
    element = ChElementBeamTimoshenko()
    element.SetNodes(nodeA, nodeB)
    element.SetSection(ChBeamSectionTimoshenko(beam_height, beam_thickness, 0.1 * beam_height ** 4 / 12, 0.1 * beam_thickness ** 4 / 12))
    beam_mesh.AddElement(element)


system.Add(beam_mesh)


fix_node = beam_mesh.GetNode(0)
fix_node.SetFixed(True)


load_node = beam_mesh.GetNode(num_elements)
load_node.SetForce(ChVector3d(0, -100, 0))


class CustomMotorFunction(ChFunction):
    def __init__(self):
        super().__init__()

    def GetVal(self, time):
        return np.sin(time)


motor_function = CustomMotorFunction()
load_node.SetForceFunction(motor_function)





application.SetTimestep(0.01)
application.SetVerbose(True)


system.SetSolver(ChSolverPSOR())
system.SetTimestepper(ChTimestepperEulerImplicitLinear())


application.Run()

chrono_types.EndChrono()