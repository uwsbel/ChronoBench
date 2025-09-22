import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys, 10, 10, 1, 1000, True, True)
ground.SetBodyFixed(True)
sys.Add(ground)


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


turtlebot = chrono.ChBodyEasyBox(sys, 0.2, 0.2, 0.1, 10, True, True)
turtlebot.SetPos(chrono.ChVectorD(0, 0.1, 0))
turtlebot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(turtlebot)


left_wheel = chrono.ChBodyEasyCylinder(sys, 0.05, 0.1, 1, True, True)
left_wheel.SetPos(chrono.ChVectorD(-0.1, 0.05, 0))
left_wheel.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(left_wheel)

right_wheel = chrono.ChBodyEasyCylinder(sys, 0.05, 0.1, 1, True, True)
right_wheel.SetPos(chrono.ChVectorD(0.1, 0.05, 0))
right_wheel.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(right_wheel)


left_motor = chrono.ChLinkMotorRotationSpeed(sys)
left_motor.Initialize(left_wheel, turtlebot)
sys.Add(left_motor)

right_motor = chrono.ChLinkMotorRotationSpeed(sys)
right_motor.Initialize(right_wheel, turtlebot)
sys.Add(right_motor)


application = chronoirr.ChIrrApp(sys, 'Turtlebot Simulation', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 0.5, 1))
application.AddLightWithShadow(chronoirr.vector3df(2, 4, 2), chronoirr.vector3df(0, 0, 0), 3, 2, 10, 120)


application.AssetBindAll()
application.AssetUpdateAll()

time = 0
while application.GetDevice().run():
    time += 0.01

    
    if time < 5:
        left_motor.SetMotorFunction(chrono.ChFunction_Const(1))
        right_motor.SetMotorFunction(chrono.ChFunction_Const(-1))
    elif time < 10:
        left_motor.SetMotorFunction(chrono.ChFunction_Const(-1))
        right_motor.SetMotorFunction(chrono.ChFunction_Const(1))
    else:
        left_motor.SetMotorFunction(chrono.ChFunction_Const(0))
        right_motor.SetMotorFunction(chrono.ChFunction_Const(0))

    sys.DoStepDynamics(0.01)
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()