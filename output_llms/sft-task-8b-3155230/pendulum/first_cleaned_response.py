import pychrono as chrono
from pychrono import irrlicht
from pychrono.irrlicht import ChVisualSystemIrrlicht

system = chrono.ChSystemNSC()
visual_system = irrlicht.ChVisualSystemIrrlicht()
visual_system.SetWindowSize(1024, 768)
visual_system.SetWindowTitle('Simple Pendulum')
visual_system.Initialize()
visual_system.AddSkyBox()
visual_system.AddCamera(chrono.ChVector3d(0, 5, 5), chrono.ChVector3d(0, 0, 0))
visual_system.AddTypicalLights()

ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, False)
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
system.Add(ground)

mass = 1.0
length = 1.0
pendulum = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, length, 0.1, mass, True, False)
pendulum.SetFixed(False)
pendulum.SetPos(chrono.ChVector3d(0, length, 0))
pendulum.SetRot(chrono.QuatFromAngleY(-chrono.CH_PI_2))
pendulum.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
system.Add(pendulum)

joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChFramed(chrono.ChVector3d(0, length, 0), chrono.QUNIT))
system.AddLink(joint)

while visual_system.Run():
    visual_system.BeginScene()
    visual_system.DrawAll()
    visual_system.EndScene()
    system.DoStepDynamics(0.01)
    angle = pendulum.GetRotation().GetAngleY()
    angular_velocity = pendulum.GetAngularVelocity().y
    print(f"Angle: {angle:.4f} rad, Angular Velocity: {angular_velocity:.4f} rad/s")