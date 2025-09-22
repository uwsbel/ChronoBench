import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.robot as robot
import math


chrono.SetChronoDataPath("../../../data/")
my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(20, 0.2, 20, 1000, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)


turtlebot = robot.TurtleBot()
turtlebot.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, 0.2, 0), 
                                     chrono.Q_from_AngZ(math.pi/2)))
turtlebot.SetCollide(True)
my_system.Add(turtlebot)


application = irr.ChIrrApp(my_system, "Turtlebot Simulation", irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(0, 4, -6))
application.AssetBindAll()
application.AssetUpdateAll()


time_step = 0.01
simulation_duration = 10  


time = 0
while time < simulation_duration:
    if time < 3:
        
        turtlebot.SetMotorSpeed(turtlebot.GetMotorLeft(), 2)
        turtlebot.SetMotorSpeed(turtlebot.GetMotorRight(), 5)
    elif time < 6:
        
        turtlebot.SetMotorSpeed(turtlebot.GetMotorLeft(), 5)
        turtlebot.SetMotorSpeed(turtlebot.GetMotorRight(), 2)
    else:
        
        turtlebot.SetMotorSpeed(turtlebot.GetMotorLeft(), 5)
        turtlebot.SetMotorSpeed(turtlebot.GetMotorRight(), 5)
    
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()
    
    time += time_step
    my_system.DoStepDynamics(time_step)


application.GetDevice().drop()