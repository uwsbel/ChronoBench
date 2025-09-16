import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


turtlebot = chrono.ChBody()
turtlebot.SetMass(10)
turtlebot.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
turtlebot.SetPos(chrono.ChVectorD(0, 0.5, 0))
turtlebot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(turtlebot)


wheel1 = chrono.ChBody()
wheel1.SetMass(1)
wheel1.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
wheel1.SetPos(chrono.ChVectorD(-0.2, 0.25, 0))
sys.Add(wheel1)

wheel2 = chrono.ChBody()
wheel2.SetMass(1)
wheel2.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
wheel2.SetPos(chrono.ChVectorD(0.2, 0.25, 0))
sys.Add(wheel2)

motor1 = chrono.ChLinkMotorRotation()
motor1.Initialize(wheel1, turtlebot, chrono.ChFrameD(chrono.ChVectorD(-0.2, 0.25, 0)))
sys.AddLink(motor1)

motor2 = chrono.ChLinkMotorRotation()
motor2.Initialize(wheel2, turtlebot, chrono.ChFrameD(chrono.ChVectorD(0.2, 0.25, 0)))
sys.AddLink(motor2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Turtlebot Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, -3))
vis.AddTypicalLights()
vis.AttachSystem(sys)


time = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if time < 2:
        motor1.SetTorque(1)
        motor2.SetTorque(1)
    elif time < 4:
        motor1.SetTorque(-1)
        motor2.SetTorque(1)
    else:
        motor1.SetTorque(1)
        motor2.SetTorque(-1)

    sys.DoStepDynamics(0.01)
    time += 0.01