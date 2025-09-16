import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.engine as engine


chrono.SetChronoDataPath('../../data/')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground_body = chrono.ChBodyEasyBox(10, 10, 0.1, 1000)
ground_body.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground_body.SetBodyFixed(True)
system.Add(ground_body)


turtlebot = veh.ChTurtlebot()
turtlebot.Init()
turtlebot.SetChassisBodyPos(chrono.ChVectorD(0, 0.5, 0))
turtlebot.SetChassisBodyRot(chrono.Q_from_AngAxis(chrono.ChVectorD(1, 0, 0), chrono.CHRONO_PI / 2))
system.Add(turtlebot.GetVehicle())


visualization = chronoirr.ChIrrApp()
visualization.AddSystem(system)
visualization.AddLogo()
visualization.AddSkyBox()
visualization.AddTypicalLights()
visualization.AddTypicalCamera(chrono.ChVectorD(0, 0.5, -1.5), chrono.ChVectorD(0, 0.5, 0))


step_size = 0.01  
end_time = 10  


start_time = system.GetChTime()
while system.GetChTime() - start_time < end_time:
    
    if system.GetChTime() < 2:
        turtlebot.SetMotorSpeed(1, 1)  
    elif system.GetChTime() < 4:
        turtlebot.SetMotorSpeed(1, -1)  
    elif system.GetChTime() < 6:
        turtlebot.SetMotorSpeed(-1, 1)  
    else:
        turtlebot.SetMotorSpeed(1, 1)  

    
    system.DoStepDynamics(step_size)

    
    visualization.BeginScene()
    visualization.Render()
    visualization.EndScene()


visualization.Close()