import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(50, 50, 0.1)
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)
system.Add(ground)


robot = turtlebot.TurtleBot()
robot.SetDriver(turtlebot.TurtleBotDriver())
initial_pos = chrono.ChVectorD(0, 0, 0.2)
initial_rot = chrono.ChQuaternionD(1, 0, 0, 0)
robot.Initialize(chrono.ChCoordsysD(initial_pos, initial_rot))
robot.AddVisualizationAssets()
robot.AddCollisionShapes()
system.Add(robot.GetChassis())


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1.5, 1), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5,5,5), chrono.ChVectorD(0,0,0), 10, 2, 10, 40, 512)


time_step = 0.01
total_time = 10.0
current_time = 0.0

while vis.Run() and current_time < total_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if current_time < 2.0:
        
        robot.SetMotorSpeeds(1.0, 1.0)
    elif current_time < 4.0:
        
        robot.SetMotorSpeeds(1.0, -0.5)
    elif current_time < 6.0:
        
        robot.SetMotorSpeeds(-0.5, 1.0)
    elif current_time < 8.0:
        
        robot.SetMotorSpeeds(-1.0, -1.0)
    else:
        
        robot.SetMotorSpeeds(0, 0)
    
    
    robot.Update()
    system.DoStepDynamics(time_step)
    current_time += time_step