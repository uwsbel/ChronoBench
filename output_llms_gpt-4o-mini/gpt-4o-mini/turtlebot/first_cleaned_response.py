import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemNSC()


chrono.ChSystemNSC.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
chrono.ChSystemNSC.Add(ground)


turtlebot = chrono.ChBody()
turtlebot.SetMass(1.0)  
turtlebot.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  
turtlebot.SetPos(chrono.ChVectorD(0, 0.1, 0))  
turtlebot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  
turtlebot.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
chrono.ChSystemNSC.Add(turtlebot)


application = chronoirr.ChIrrApp(chrono.ChSystemNSC, "Turtlebot Simulation", chronoirr.dimension2d(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0, 0))


time_step = 0.01  
simulation_time = 10.0  
current_time = 0.0


while current_time < simulation_time:
    
    chrono.ChSystemNSC.DoStepDynamics(time_step)

    
    if 2.0 < current_time < 4.0:  
        turtlebot.SetPos(turtlebot.GetPos() + chrono.ChVectorD(-0.1, 0, 0))
    elif 6.0 < current_time < 8.0:  
        turtlebot.SetPos(turtlebot.GetPos() + chrono.ChVectorD(0.1, 0, 0))

    
    application.DrawAll()
    application.DoStep()

    
    current_time += time_step


application.AssetBindAll()
application.AssetUpdateAll()
application.GetDevice().drop()