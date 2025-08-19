import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea
import numpy as np
import math as m


sys = chrono.ChSystemNSC()


material = chrono.ChContactMaterialNSC()
material.SetFriction(0.5)


class MyMotor (chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)
    def GetVal(self, x):
        return 0.5 * m.sin(3 * x)


ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)


beam = fea.ChBeam3dElement()
beam.SetDensity(1000)
beam.SetSection(0.1, 0.1, 0.1, 0.1, 0.01, 0.01)
beam.SetFixed(False)  
beam.SetNodesOwnCoords(
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1)),
    chrono.ChFramed(chrono.ChVector3d(1, 0, 0), chrono.ChMatrix33d(1)))
sys.Add(beam)



rot1 = chrono.ChLinkLockRevolute()
rot1.Initialize(ground, beam.GetNode1(), chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1)))
sys.Add(rot1)



rigidBody = chrono.ChBody()
rigidBody.SetMass(1)
rigidBody.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
rigidBody.SetPos(chrono.ChVector3d(1, 0, 0))
sys.Add(rigidBody)

rot2 = chrono.ChLinkLockRevolute()
rot2.Initialize(beam.GetNode2(), rigidBody, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1)))
sys.Add(rot2)


motor = chrono.ChFunctionSetpointSquarewave(my_motor, 0, 0.5)
motor2 = chrono.ChLinkMotorRotation(beam.GetNode2(), rigidBody, rot2, motor, True, True)
sys.Add(motor2)


sphere = chrono.ChBodyEasyBox(0.1, 10, 0.1, 1000, True, False)
sphere.SetPos(chrono.ChVector3d(1, -0.1, 0))
sphere.SetFixed(True)
sys.Add(sphere)
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


cube = chrono.ChBodyEasyBox(5, 1, 5, 1000, True, False)
cube.SetPos(chrono.ChVector3d(0, -0.5, 0))
cube.SetFixed(True)
cube.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
sys.Add(cube)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Test')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, 2, 2), chrono.ChVector3d(1, 0, 0))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)