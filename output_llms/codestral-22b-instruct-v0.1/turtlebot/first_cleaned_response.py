import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
system.Add(ground)


turtlebot = chrono.ChBodyEasyBox(0.2, 0.1, 0.4, 10, True, True)
turtlebot.SetPos(chrono.ChVectorD(0, 0.5, 0))
turtlebot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(turtlebot)


app = irr.ChIrrApp(system, 'Turtlebot Simulation', irr.dimension2du(800, 600))
app.AddTypicalLogo()
app.AddTypicalCamera(irr.vector3df(0, 1, -2))
app.AddTypicalLights()


time_step = 0.01
total_time = 10
turn_duration = 2
left_turn_start = 2
right_turn_start = 5

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()

    current_time = system.GetChTime()
    if current_time < total_time:
        if current_time < left_turn_start:
            
            turtlebot.SetWvel_par(chrono.ChVectorD(0, 0, 1))
        elif current_time < left_turn_start + turn_duration:
            
            turtlebot.SetWvel_par(chrono.ChVectorD(0, 1, 0))
        elif current_time < right_turn_start:
            
            turtlebot.SetWvel_par(chrono.ChVectorD(0, 0, 1))
        elif current_time < right_turn_start + turn_duration:
            
            turtlebot.SetWvel_par(chrono.ChVectorD(0, -1, 0))
        else:
            
            turtlebot.SetWvel_par(chrono.ChVectorD(0, 0, 1))

        system.DoStepDynamics(time_step)

app.GetDevice().closeDevice()