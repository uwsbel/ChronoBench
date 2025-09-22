```python
import pychrono as chrono
import pychrono.irrlicht as chronoirr








system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)








robot = chrono.ChRobot(system)
robot.ReadFromFile("turtlebot.obj")


robot.SetPos(chrono.ChVectorD(0, 0.5, 0))
robot.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 4, chrono.ChVectorD(0, 0, 1)))








vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Turtlebot Simulation")


camera = vis.GetCamera()
camera.SetPosition(chrono.ChVectorD(5, 5, 3))
camera.SetLookAt(chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChLight(chrono.ChVectorD(2, 5, 2)))








timestep = 0.01  

while vis.Run():
    
    if system.GetChTime() < 5:
        robot.SetMotorSpeed(0, 10)  
        robot.SetMotorSpeed(1, -10)  
    else:
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(1, 0)

    
    system.DoStepDynamics(timestep)

    
    vis.Render()