import chrono as ch
import numpy as np


ch.ChEngine.Initialize()


system = ch.ChSystemNSC()


system.Set_G_acc(ch.ChVector3d(0, -9.81, 0))


beam_length = 10.0
beam_width = 0.1
beam_height = 0.1
num_elements = 10

fea_mesh = ch.ChMesh()


for i in range(num_elements + 1):
    node = ch.ChNodeFEA(beam_length / num_elements * i, 0, 0)
    fea_mesh.AddNode(node)


for i in range(num_elements):
    elem = ch.ChElementBeamTimoshenko(
        fea_mesh.GetNode(i),
        fea_mesh.GetNode(i + 1),
        ch.ChVector3d(beam_width, beam_height, 0),
        ch.ChVector3d(0, 0, 0),
        0.1e6,
        0.3,
        0.001,
    )
    fea_mesh.AddElement(elem)


system.Add(fea_mesh)


ground = ch.ChBody()
ground.SetPos(ch.ChVector3d(0, 0, 0))
system.Add(ground)


constraint = ch.ChLinkMate(
    fea_mesh.GetNode(0),
    ground,
    ch.ChLinkMate.Revolute,
    ch.ChVector3d(0, 0, 0),
    ch.ChVector3d(0, 0, 0),
    ch.ChVector3d(0, 1, 0),
)
system.Add(constraint)


class CustomMotorFunction(ch.ChFunction):
    def __init__(self):
        super().__init__()

    def Get_y(self, t):
        return np.sin(t)


motor = ch.ChLinkMotor(
    fea_mesh.GetNode(num_elements),
    ground,
    ch.ChLinkMotor.Rotational,
    ch.ChVector3d(beam_length, 0, 0),
    ch.ChVector3d(0, 0, 0),
    CustomMotorFunction(),
)
system.Add(motor)


vis = ch.ChVisualization(system)
vis.SetAllVisible(True)


app = ch.ChIrrApp(vis, "Beam Buckling Simulation")


system.SetSolver(ch.ChSolverPSBM())
system.SetTimestepper(ch.ChTimestepperEulerImplicitLinear())


app.Run()