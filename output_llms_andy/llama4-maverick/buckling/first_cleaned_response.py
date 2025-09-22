import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)


beam_length = 1.0
beam_elements = 10
beam_section = fea.ChBeamSectionEulerAdvanced()
beam_section.SetDensity(1000)
beam_section.SetYoungModulus(2e6)
beam_section.SetPoissonRatio(0.3)
beam_section.SetBeamRaleyghDamping(0.01)


node_start = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
mesh.AddNode(node_start)
for i in range(1, beam_elements + 1):
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * beam_length / beam_elements, 0, 0)))
    mesh.AddNode(node)
    elem = fea.ChElementBeamEuler(node_start, node, beam_section)
    mesh.AddElement(elem)
    node_start = node


constraint_pos = chrono.ChLinkMateGeneric()
constraint_pos.Initialize(node_start, sys.NewBody(), False, node_start.GetFrame_COG(), node_start.GetFrame_COG())
sys.Add(constraint_pos)


class MotorFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()

    def Get_y(self, x):
        return np.sin(x)

motor_func = MotorFunction()


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(mesh.GetNode(0).GetBody(), mesh.GetNode(beam_elements).GetBody(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetAngleFunction(motor_func)
sys.Add(motor)


sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


application = chronoirr.ChIrrApp(sys, "Beam Buckling Simulation", chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0.5, 0.5, 1.0))
application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    sys.DoStepDynamics(0.01)
    application.EndScene()