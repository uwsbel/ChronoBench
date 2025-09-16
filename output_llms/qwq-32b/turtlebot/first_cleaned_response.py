import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.turtlebot as turtlebot


my_system = chrono.ChSystemNSC()
my_system.SetGravity(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground_shape = chrono.ChBox(chrono.ChVectorD(10, 0.1, 10))
ground.AddAsset(ground_shape)
ground.AddAsset(chrono.ChColorAsset(0.2, 0.3, 0.5))
my_system.Add(ground)


my_turtlebot = turtlebot.Turtlebot()
my_turtlebot.Initialize(my_system)
my_turtlebot.SetPos(chrono.ChVectorD(0, 0.1, 0))  
my_turtlebot.SetRot(chrono.Q_from_Euler_123(0, 0, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Turtlebot Simulation')
vis.SetCameraPosition(chrono.ChVectorD(2, 2, 5))  
vis.SetCameraLookat(chrono.ChVectorD(0, 0, 0))
vis.SetLightIntensity(0.8)
my_system.SetVisualSystem(vis)


step_size = 1e-3  
time = 0.0


while vis.Run():
    time = my_system.GetChTime()
    
    
    if time < 2.0:
        my_turtlebot.SetMotorSpeeds(5, 5)  
    elif time < 4.0:
        my_turtlebot.SetMotorSpeeds(-5, 5)  
    elif time < 6.0:
        my_turtlebot.SetMotorSpeeds(5, -5)  
    else:
        my_turtlebot.SetMotorSpeeds(0, 0)  
    
    
    my_system.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()