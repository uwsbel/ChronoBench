import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("C:/path/to/chrono/data/")


my_system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(my_system, "Epicyclic Gears Simulation", chronoirr.dimension2du(800, 600))


application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 1), chronoirr.vector3df(0, 0, 0))
application.AddTypicalLights()


truss_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
truss_body.SetPos(chrono.ChVectorD(0, 0, 0))
truss_body.SetBodyFixed(True)
my_system.Add(truss_body)


bar_body = chrono.ChBodyEasyCylinder(0.1, 1, 100, True, True)
bar_body.SetPos(chrono.ChVectorD(0, 0, 0.5))
bar_body.SetRot(chrono.ChQuaternionD(0, 0, 0, 1))
my_system.Add(bar_body)


gear1_body = chrono.ChBodyEasyCylinder(0.2, 0.5, 100, True, True)
gear1_body.SetPos(chrono.ChVectorD(0.5, 0, 0.5))
gear1_body.SetRot(chrono.ChQuaternionD(0, 0, 0, 1))
my_system.Add(gear1_body)

gear2_body = chrono.ChBodyEasyCylinder(0.1, 0.2, 100, True, True)
gear2_body.SetPos(chrono.ChVectorD(-0.5, 0, 0.5))
gear2_body.SetRot(chrono.ChQuaternionD(0, 0, 0, 1))
my_system.Add(gear2_body)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar_body, gear1_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(0, 0, 0, 1)))
motor.SetMotorFunction(chrono.ChFunction_Const(chrono.CH_C_PI / 2))
my_system.Add(motor)


constraint1 = chrono.ChLinkRevolute()
constraint1.Initialize(gear1_body, gear2_body, chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0.5), chrono.ChQuaternionD(0, 0, 0, 1)))
my_system.Add(constraint1)

constraint2 = chrono.ChLinkRevolute()
constraint2.Initialize(gear2_body, bar_body, chrono.ChFrameD(chrono.ChVectorD(-0.5, 0, 0.5), chrono.ChQuaternionD(0, 0, 0, 1)))
my_system.Add(constraint2)


truss_body.AddAsset(chrono.ChColorAsset(0.5, 0.5, 0.5))
bar_body.AddAsset(chrono.ChColorAsset(1, 0, 0))
gear1_body.AddAsset(chrono.ChColorAsset(0, 1, 0))
gear2_body.AddAsset(chrono.ChColorAsset(0, 0, 1))


application.AssetBindAll()
application.AssetUpdateAll()
application.Simulate(0, 10)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()