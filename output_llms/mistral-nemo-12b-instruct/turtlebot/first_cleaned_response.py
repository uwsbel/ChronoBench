import pychrono as chrono
import pychrono.irrlicht as irr


chrono.Initialize()


system = chrono.ChSystemNSC()
system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(10, 10, 1, 1000, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
system.Add(ground)


turtlebot = chrono.ChBodyEasyCylinder(0.1, 0.15, 1000, True)
turtlebot.SetPos(chrono.ChVectorD(0, 0.75, 0))
turtlebot.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI / 2))
system.Add(turtlebot)


left_wheel = chrono.ChLinkMotor()
left_wheel.Initialize(turtlebot, ground, chrono.ChFrameD(chrono.ChVectorD(0.05, 0.1, 0.1), chrono.Q_from_AngX(chrono.CH_C_PI / 2)))
left_wheel.SetSpdLimit(-5, 5)
system.Add(left_wheel)

right_wheel = chrono.ChLinkMotor()
right_wheel.Initialize(turtlebot, ground, chrono.ChFrameD(chrono.ChVectorD(-0.05, 0.1, 0.1), chrono.Q_from_AngX(chrono.CH_C_PI / 2)))
right_wheel.SetSpdLimit(-5, 5)
system.Add(right_wheel)


app = irr.ChIrrApp(system, 'Turtlebot Simulation')
app.SetCamera(chrono.ChVectorD(1, 1, -5), chrono.ChVectorD(0, 0, 0))
app.AddLightWithDir(chrono.ChVectorD(3, 3, -5), chrono.ChVectorD(0, 0, -1))


while app.GetDevice().run():
    
    system.DoStepDynamics(0.01)

    
    time = app.GetTime()
    if time > 2 and time < 4:
        left_wheel.SetTargetSpeed(1)
        right_wheel.SetTargetSpeed(-1)
    elif time > 6 and time < 8:
        left_wheel.SetTargetSpeed(-1)
        right_wheel.SetTargetSpeed(1)
    else:
        left_wheel.SetTargetSpeed(0)
        right_wheel.SetTargetSpeed(0)

    
    app.BeginScene(True, True, chrono.ChColor(0.3, 0.3, 0.3))
    app.DrawAll()
    app.EndScene()
    app.GetDevice().display()