import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasySphere(10, 10)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
system.AddBody(ground)


turtlebot = chrono.ChBodyEasyCylinder(0.2, 0.2, 0.1)
turtlebot.SetPos(chrono.ChVectorD(0, 0, 0))
turtlebot.SetRot(chrono.Q_from_Euler(chrono.ChVectorD(0, 0, 0)))
system.AddBody(turtlebot)


vis = chronoirr.ChIrrApp(system, 'Turtlebot Simulation')
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.SetCamera(vis.GetActiveCamera())
vis.SetSunLight(chrono.ChVectorD(1, 1, 1))


dt = 0.01
while True:
    
    if vis.GetTime() > 1:
        turtlebot.SetBodyFixedRotation(chrono.ChQuaternionD(0, 0, 0, 1))  
        turtlebot.SetBodyFixedRotation(chrono.ChQuaternionD(0, 0, 0.1, 1))  
    if vis.GetTime() > 3:
        turtlebot.SetBodyFixedRotation(chrono.ChQuaternionD(0, 0, 0, 1))  
        turtlebot.SetBodyFixedRotation(chrono.ChQuaternionD(0, 0, -0.1, 1))  

    
    system.DoStepDynamics(dt)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()