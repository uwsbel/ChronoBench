import pychrono as chrono
import pychrono.robot as turtlebot
import math


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  


ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True)  
ground.SetPos(chrono.ChVector3d(0, 0, -1))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  
system.Add(ground)  


robot = turtlebot.TurtleBot(system)  
robot.Initialize(chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))  


vis = chrono.ChVisualSystemIrrlicht()  
vis.AttachSystem(system)  
vis.SetWindowSize(1280, 720)  
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 0))  
vis.AddTypicalLights()  


time_step = 1e-3


time = 0
while vis.Run():
    time += time_step  

    
    if time < 2.0:
        robot.SetMotorSpeed(0, 0)  
    elif time < 4.0:
        robot.SetMotorSpeed(1.0, 0)  
    elif time < 6.0:
        robot.SetMotorSpeed(0, 1.0)  
    else:
        robot.SetMotorSpeed(0, 0)  

    
    robot.Update()
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    system.DoStepDynamics(time_step)