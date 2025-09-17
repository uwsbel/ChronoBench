import chrono
import chrono.irrlicht as chronoirr
import chrono.robotics as chronorobotics





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  





body_ground = chrono.ChBodyEasy()
body_ground.SetBodyFixed(True)
body_ground.SetMass(1000)
body_ground.SetShape(chrono.ChBoxShape())
body_ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
body_ground.SetVisualizationType(chrono.ChVisualizationType.BOX)
system.Add(body_ground)






turtlebot = chronorobotics.ChTurtlebot()
turtlebot.SetWorkingMemoryName("Turtlebot")


turtlebot.SetPos(chrono.ChVectorD(0, 0.1, 0))
turtlebot.SetTransform(chrono.ChQuaternionD(0, 0, 0))


system.Add(turtlebot.GetSystem())





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Turtlebot Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 1.5, -3))
vis.AddTypicalLights()





time_left_turn = 5.0  
time_right_turn = 10.0  
turn_speed = 0.5  

simulation_time = 20.0
time_step = 0.01

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    current_time = system.GetChTime()

    
    if current_time >= time_left_turn and current_time < time_right_turn:
        
        turtlebot.SetLeftWheelSpeed(turn_speed)
        turtlebot.SetRightWheelSpeed(-turn_speed)
    elif current_time >= time_right_turn:
        
        turtlebot.SetLeftWheelSpeed(-turn_speed)
        turtlebot.SetRightWheelSpeed(turn_speed)
    else:
        
        turtlebot.SetLeftWheelSpeed(0.2)
        turtlebot.SetRightWheelSpeed(0.2)

    if current_time > simulation_time:
        break