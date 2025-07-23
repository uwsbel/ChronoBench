import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


beam_wy = 0.1
beam_wz = 0.1
beam_L = 10.0
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)


sys.Add(mesh)


body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)
sys.Add(body_floor)


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(hnode1, body_floor, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0)))
sys.Add(rev_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(hnode2, body_floor, chrono.ChCoordsysD(chrono.ChVector3d(beam_L, 0, 0)))
sys.Add(motor)


class CustomMotorFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()

    def Get_y(self, x):
        return np.sin(x)

    def Clone(self):
        return CustomMotorFunction()

motor_fun = CustomMotorFunction()
motor.SetSpeedFunction(motor_fun)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, 5))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()