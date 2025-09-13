import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data")
my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)


turtlebot = chrono.ChBodyEasyBox(0.2, 0.1, 0.2, 10, True, True)
turtlebot.SetPos(chrono.ChVectorD(0, 0, 0))
turtlebot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
my_system.Add(turtlebot)


turtlebot.SetPos(chrono.ChVectorD(0, 0, 0))
turtlebot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


left_motor = chrono.ChLinkMotorRotationSpeed()
right_motor = chrono.ChLinkMotorRotationSpeed()


left_motor.Initialize(turtlebot, None, chrono.ChVectorD(0.1, 0, 0), chrono.ChVectorD(0, 1, 0))
right_motor.Initialize(turtlebot, None, chrono.ChVectorD(-0.1, 0, 0), chrono.ChVectorD(0, 1, 0))

my_system.Add(left_motor)
my_system.Add(right_motor)


application = chronoirr.ChIrrApp(my_system, "Turtlebot Simulation", chronoirr.dimension2du(800, 600), False, True)


application.GetCamera()->SetPosition(chrono.ChVectorD(0, 5, -5))
application.GetCamera()->SetFocalPoint(chrono.ChVectorD(0, 0, 0))


application.AddLightWithShadow(chrono.ChVectorD(5, 5, -5), chrono.ChVectorD(0, -1, 0), 10, 50, 50, 512, 2048)
application.AddTypicalLights()


timestep = 0.01


left_speed = 10
right_speed = 10


turn_start = 2
turn_end = 4


while application.GetDevice().run():
    if application.GetDevice().isWindowActive():
        
        if application.GetSystem()->GetChTime() >= turn_start and application.GetSystem()->GetChTime() <= turn_end:
            left_speed = 10
            right_speed = -10
        else:
            left_speed = 10
            right_speed = 10

        
        left_motor.Set_speed_dt(left_speed * timestep)
        right_motor.Set_speed_dt(right_speed * timestep)

        
        my_system.DoStepDynamics(timestep)
        application.BeginScene()
        application.DrawAll()
        application.EndScene()