import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("C:/path/to/chrono/data/")


my_system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(my_system, "Epicyclic Gears Simulation", chronoirr.dimension2du(800, 600))


application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 1.5))
application.AddTypicalLights()
application.AddLightWithShadow(chronoirr.vector3df(2, 4, 2), chronoirr.vector3df(0, 0, 0), 3, 2, 2, 30, 512)


truss_body = chrono.ChBodyEasyBox(1, 0.1, 0.1, 1000, True)
truss_body.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(truss_body)


bar_body = chrono.ChBodyEasyBox(0.1, 1, 0.1, 1000, True)
bar_body.SetPos(chrono.ChVectorD(0, 0, 0.5))
bar_body.SetRot(chrono.ChQuaternionD(chrono.Q_from_AngX(chrono.CH_C_PI / 4)))
my_system.Add(bar_body)


gear1_body = chrono.ChBodyEasyCylinder(0.2, 0.1, 1000, True)
gear1_body.SetPos(chrono.ChVectorD(0.5, 0, 0))
gear1_body.SetRot(chrono.ChQuaternionD(chrono.Q_from_AngX(chrono.CH_C_PI / 2)))
my_system.Add(gear1_body)

gear2_body = chrono.ChBodyEasyCylinder(0.1, 0.2, 1000, True)
gear2_body.SetPos(chrono.ChVectorD(-0.5, 0, 0))
gear2_body.SetRot(chrono.ChQuaternionD(chrono.Q_from_AngX(chrono.CH_C_PI / 2)))
my_system.Add(gear2_body)


gear_motor = chrono.ChLinkMotorRotationSpeed()
gear_motor.Initialize(gear1_body, gear2_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
gear_motor.SetSpindleConstraint(chrono.ChLinkMotorRotationSpeed.SpindleConstraint_SCREW)
gear_motor.SetMotorFunction(chrono.ChFunction_Ramp(0, 10))
my_system.Add(gear_motor)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(bar_body, gear_motor.GetBody2(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))
my_system.Add(revolute_joint)


revolute_joint2 = chrono.ChLinkRevolute()
revolute_joint2.Initialize(gear_motor.GetBody2(), truss_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
my_system.Add(revolute_joint2)


application.SetTimestep(0.01)
application.SetEndTime(10)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()