import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("data/")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
system.Add(ground)



turtlebot = chrono.ChBody()
turtlebot.SetPos(chrono.ChVectorD(0, 0.5, 0))
turtlebot.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 4, chrono.ChVectorD(0, 0, 1)))
system.Add(turtlebot)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
camera = vis.GetCamera()
camera.SetLocation(chrono.ChVectorD(2, 2, 2))
camera.SetLookAt(chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChLightPoint(chrono.ChVectorD(2, 2, 2)))


while vis.Run():
    
    time = system.GetChTime()
    if time < 5:
        turtlebot.SetBodyFixed(False)  
        
    else:
        turtlebot.SetBodyFixed(True)  

    
    system.DoStepDynamics(0.01)
    vis.Render()