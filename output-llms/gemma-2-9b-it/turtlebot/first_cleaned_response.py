import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(10, 10, 0.1)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


turtlebot = chrono.ChBodyEasySphere(0.2, 0.1)
turtlebot.SetPos(chrono.ChVectorD(0, 0, 0.1))
turtlebot.SetRot(chrono.Q_from_Euler(chrono.ChVectorD(0, 0, 0)))  
system.Add(turtlebot)


vis = chronoirr.ChIrrApp(system, 'Turtlebot Simulation')
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.SetSunLight(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))


motor_speed = 0.1
turn_time = 2.0


while vis.Run():
    
    current_time = vis.GetChTime()
    if current_time % turn_time < turn_time / 2:
        turtlebot.SetBodyFixed(True)
        turtlebot.AddTorque(chrono.ChVectorD(0, 0, motor_speed))
    else:
        turtlebot.SetBodyFixed(True)
        turtlebot.AddTorque(chrono.ChVectorD(0, 0, -motor_speed))

    
    vis.BeginScene()
    system.DoStepDynamics(1/240.0)  
    vis.DrawAll()
    vis.EndScene()