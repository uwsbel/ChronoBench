import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.turtlebot as turtlebot


my_system = chrono.ChSystemNSC()
my_system.SetGravity(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground_shape = chrono.ChBox(chrono.ChVectorD(10, 0.1, 10))
ground.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0.8, 0.8)))
ground.GetCollisionModel().AddBox(10, 0.1, 10)
ground.SetCollide(True)
my_system.Add(ground)


turtle = turtlebot.Turtlebot()
turtle.Initialize()
turtle.SetPos(chrono.ChVectorD(0, 0.1, 0))  
turtle.SetRot(chrono.Q_from_Euler123(chrono.ChVectorD(0, 0, 0)))
my_system.Add(turtle.GetBody())


visualiz = chronoirr.ChIrrApp(my_system, 'Turtlebot Simulation', chronoirr.dimension2du(800, 600))
visualiz.AddTypicalLights()
visualiz.AddAsset(chronoirr.ChIrrTools.getChronoLogo())
visualiz.SetCamera(chronoirr.ChVectorD(3, 3, 3), chronoirr.ChVectorD(0, 0, 0))
visualiz.SetTimestep(0.01)
visualiz.SetTryRealtime(True)
visualiz.Initialize()


while visualiz.Run():
    current_time = my_system.GetChTime()
    
    
    if 2.0 <= current_time < 4.0:
        turtle.GetLeftMotor().SetSpeedTarget(-5)  
        turtle.GetRightMotor().SetSpeedTarget(5)
    elif 4.0 <= current_time < 6.0:
        turtle.GetLeftMotor().SetSpeedTarget(5)   
        turtle.GetRightMotor().SetSpeedTarget(-5)
    else:
        turtle.GetLeftMotor().SetSpeedTarget(5)  
        turtle.GetRightMotor().SetSpeedTarget(5)
    
    my_system.DoStepDynamics(visualiz.GetTimestep())
    visualiz.Advance()