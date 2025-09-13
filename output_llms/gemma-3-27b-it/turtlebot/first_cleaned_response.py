import chrono
import chrono.irrlicht as chronoirr
import chrono.robotics as chronorobotics





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  





body_floor = chrono.ChBodyEasy()
body_floor.SetBodyFixed(True)
body_floor.SetMass(1000)
body_floor.SetPos(chrono.ChVectorD(0, -0.1, 0))
body_floor.SetShapeBoxes(0.5, 0.1, 0.5)
system.Add(body_floor)






turtlebot = chronorobotics.ChTurtlebot()
turtlebot.SetPos(chrono.ChVectorD(0, 0.2, 0))
turtlebot.SetTransform(chrono.ChQuaternionD(0, 0, 0, 1))
system.Add(turtlebot)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Turtlebot Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 1.5, -3))
vis.AddTypicalLights()





simulation_time = 10  
time_step = 0.01  
current_time = 0

turn_left_time = 2.0
turn_right_time = 6.0
turn_duration = 2.0
turn_speed = 0.5  

while current_time < simulation_time:
    
    if current_time >= turn_left_time and current_time < turn_left_time + turn_duration:
        turtlebot.SetLeftWheelSpeed(turn_speed)
        turtlebot.SetRightWheelSpeed(-turn_speed)
    elif current_time >= turn_right_time and current_time < turn_right_time + turn_duration:
        turtlebot.SetLeftWheelSpeed(-turn_speed)
        turtlebot.SetRightWheelSpeed(turn_speed)
    else:
        turtlebot.SetLeftWheelSpeed(0)
        turtlebot.SetRightWheelSpeed(0)

    
    system.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    current_time += time_step





vis.Close()