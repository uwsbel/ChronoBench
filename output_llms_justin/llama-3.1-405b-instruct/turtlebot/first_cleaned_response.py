import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(system, 10, 10, 0.1, 1000, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
system.Add(ground)


turtlebot = chrono.ChBodyEasyBox(system, 0.2, 0.2, 0.1, 10, True)
turtlebot.SetPos(chrono.ChVectorD(0, 0.1, 0))
turtlebot.SetRot(chrono.Q_from_AngZ(chrono.CH_C_PI / 4))
system.Add(turtlebot)


left_wheel = chrono.ChBodyEasyCylinder(system, 0.05, 0.1, 1, True)
left_wheel.SetPos(chrono.ChVectorD(-0.1, 0.05, 0))
left_wheel.SetRot(chrono.Q_from_AngZ(chrono.CH_C_PI / 2))
turtlebot.AddChild(left_wheel)

right_wheel = chrono.ChBodyEasyCylinder(system, 0.05, 0.1, 1, True)
right_wheel.SetPos(chrono.ChVectorD(0.1, 0.05, 0))
right_wheel.SetRot(chrono.Q_from_AngZ(-chrono.CH_C_PI / 2))
turtlebot.AddChild(right_wheel)


left_motor = chrono.ChLinkMotorRotationSpeed(system)
left_motor.Initialize(left_wheel, turtlebot)
left_motor.SetMotorFunction(chrono.ChFunction_Const(chrono.CH_C_PI))
system.Add(left_motor)

right_motor = chrono.ChLinkMotorRotationSpeed(system)
right_motor.Initialize(right_wheel, turtlebot)
right_motor.SetMotorFunction(chrono.ChFunction_Const(-chrono.CH_C_PI))
system.Add(right_motor)


application = chronoirr.ChIrrApp(system, "Turtlebot Simulation", chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 1))
application.AddLightWithShadow(chronoirr.vector3df(1, 1, 1), chronoirr.vector3df(0, 0, 0), 3, 3, 3, 3, 128)


application.AssetBindAll()
application.AssetUpdateAll()
application.AddShadowAll()

time = 0
while application.GetDevice().run():
    time += 0.01
    if time < 5:
        left_motor.SetMotorFunction(chrono.ChFunction_Const(chrono.CH_C_PI))
        right_motor.SetMotorFunction(chrono.ChFunction_Const(-chrono.CH_C_PI))
    elif time < 10:
        left_motor.SetMotorFunction(chrono.ChFunction_Const(-chrono.CH_C_PI))
        right_motor.SetMotorFunction(chrono.ChFunction_Const(chrono.CH_C_PI))
    else:
        left_motor.SetMotorFunction(chrono.ChFunction_Const(0))
        right_motor.SetMotorFunction(chrono.ChFunction_Const(0))

    system.DoStepDynamics(0.01)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()