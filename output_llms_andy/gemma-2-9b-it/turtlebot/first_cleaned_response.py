import pychrono as chrono
import pychrono.irrlicht as vis


chrono.SetChronoData(chrono.CHRONO_DATA_USE_MULTITHREADING)
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(10, 10, 0.1)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.AddBody(ground)


turtlebot = chrono.ChBodyEasyCylinder(0.3, 0.3, 0.1)
turtlebot.SetPos(chrono.ChVectorD(0, 0, 0.1))
turtlebot.SetRot(chrono.Q_from_Euler(chrono.CH_C_DEGREE, 0, 0))  
system.AddBody(turtlebot)


vis.Initialize()
vis.Add(turtlebot)
vis.SetCamera(
    chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0)
)
vis.SetAmbientLight(chrono.ChColor(0.3, 0.3, 0.3))
vis.SetDirectionalLight(chrono.ChColor(0.8, 0.8, 0.8))


dt = 0.01
while True:
    
    if chrono.GetChTime() > 1:
        turtlebot.SetWvel(chrono.ChVectorD(0.5, 0, 0))  
    elif chrono.GetChTime() > 2:
        turtlebot.SetWvel(chrono.ChVectorD(-0.5, 0, 0))  

    
    system.DoStepDynamics(dt)

    
    vis.Render()

    
    chrono.Sleep(dt)