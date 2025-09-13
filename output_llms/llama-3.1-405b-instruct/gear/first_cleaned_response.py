import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


gears_radius = [0.2, 0.1]  
gears_thickness = 0.05  
bar_length = 0.5  
truss_length = 1.0  
motor_speed = 2.0  


my_system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(my_system, 'Epicyclic Gears Simulation', chronoirr.dimension2du(800, 600))


truss_body = chrono.ChBodyEasyBox(my_system, truss_length, 0.1, 0.1, 1000, True)
truss_body.SetPos(chrono.ChVectorD(0, 0, 0))
truss_body.SetBodyFixed(True)
my_system.Add(truss_body)


bar_body = chrono.ChBodyEasyBox(my_system, bar_length, 0.05, 0.05, 100, False)
bar_body.SetPos(chrono.ChVectorD(0, 0, 0.5))
bar_body.SetMass(10)
my_system.Add(bar_body)


gear1_body = chrono.ChBodyEasyCylinder(my_system, gears_radius[0], gears_thickness, 100, False)
gear1_body.SetPos(chrono.ChVectorD(-0.25, 0, 0.5))
gear1_body.SetMass(5)
my_system.Add(gear1_body)

gear2_body = chrono.ChBodyEasyCylinder(my_system, gears_radius[1], gears_thickness, 100, False)
gear2_body.SetPos(chrono.ChVectorD(0.25, 0, 0.5))
gear2_body.SetMass(5)
my_system.Add(gear2_body)


motor = chrono.ChLinkMotorRotationSpeed(my_system)
motor.Initialize(gear1_body, gear2_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5)))
motor.SetMotorFunction(chrono.ChFunction_Ramp(0, motor_speed))
my_system.Add(motor)


bar_gear1_joint = chrono.ChLinkRevolute(my_system)
bar_gear1_joint.Initialize(bar_body, gear1_body, chrono.ChFrameD(chrono.ChVectorD(-0.25, 0, 0.5)))
my_system.Add(bar_gear1_joint)

bar_gear2_joint = chrono.ChLinkRevolute(my_system)
bar_gear2_joint.Initialize(bar_body, gear2_body, chrono.ChFrameD(chrono.ChVectorD(0.25, 0, 0.5)))
my_system.Add(bar_gear2_joint)


truss_bar_joint = chrono.ChLinkRevolute(my_system)
truss_bar_joint.Initialize(truss_body, bar_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5)))
my_system.Add(truss_bar_joint)


application.AssetBindAll()
application.AssetUpdateAll()


application.SetCamera(chronoirr.vector3df(1, 1, 1), chronoirr.vector3df(0, 0, 0))


application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()