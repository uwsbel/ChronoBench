import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
system.Add(ground)


turtlebot = chrono.ChBodyEasyBox(0.2, 0.1, 0.4, 10, True, True)
turtlebot.SetPos(chrono.ChVectorD(0, 0.2, 0))
turtlebot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(turtlebot)


app = irr.ChIrrApp(system, 'Turtlebot Simulation', irr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalCamera(irr.vector3df(0, 1, -2))
app.AddTypicalLights()


step_size = 0.01


while app.GetDevice().run():
    
    time = system.GetChTime()
    if time < 2:
        
        turtlebot.SetWvel_par(chrono.ChVectorD(0, 0, 3.14))
    elif time < 4:
        
        turtlebot.SetWvel_par(chrono.ChVectorD(0, 0, -3.14))
    else:
        
        turtlebot.SetWvel_par(chrono.ChVectorD(0, 0, 0))

    
    system.DoStepDynamics(step_size)

    
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()