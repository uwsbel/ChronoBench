import pychrono as chrono
import pychrono.irrlicht as chronoirr


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


wheel_L = chrono.ChBody()
wheel_L.SetMass(1)
wheel_L.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
wheel_L.SetPos(chrono.ChVectorD(-0.5, 0.25, 0))
sys.Add(wheel_L)

wheel_R = chrono.ChBody()
wheel_R.SetMass(1)
wheel_R.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
wheel_R.SetPos(chrono.ChVectorD(0.5, 0.25, 0))
sys.Add(wheel_R)


motor_L = chrono.ChLinkMotorRotationSpeed()
motor_L.Initialize(turtlebot, wheel_L, chrono.ChFrameD(chrono.ChVectorD(-0.5, 0.25, 0)))
sys.Add(motor_L)

motor_R = chrono.ChLinkMotorRotationSpeed()
motor_R.Initialize(turtlebot, wheel_R, chrono.ChFrameD(chrono.ChVectorD(0.5, 0.25, 0)))
sys.Add(motor_R)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Turtlebot Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, -4))
vis.AddTypicalLights()
vis.AttachSystem(sys)


time = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    time += 0.01
    
    
    if time < 2:
        motor_L.SetSpeed(1)
        motor_R.SetSpeed(1)
    elif time < 4:
        motor_L.SetSpeed(-1)
        motor_R.SetSpeed(1)
    else:
        motor_L.SetSpeed(1)
        motor_R.SetSpeed(-1)
        
    vis.EndScene()