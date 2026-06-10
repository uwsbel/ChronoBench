import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import math






system = chrono.ChSystemNSC()


system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)






ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)


ground = chrono.ChBodyEasyBox(
    20, 20, 1,        
    1000,             
    True,             
    True,             
    ground_mat        
)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))   
ground.SetFixed(True)                          


ground_texture = chrono.ChVisualMaterial()
ground_texture.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.GetVisualShape(0).SetMaterial(0, ground_texture)

system.Add(ground)






init_pos = chrono.ChVector3d(0, 0, 0.0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   


turtlebot = robot.TurtleBot(system, init_pos, init_rot)


turtlebot.Initialize()





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()


vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.5),  
              chrono.ChVector3d(0, 0, 0))       


vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),  
    chrono.ChVector3d(0, 0, 0),         
    3,                                   
    2, 6,                                
    50                                   
)
vis.EnableShadows()





time_step = 0.005   
time = 0.0

print("Starting Turtlebot simulation...")

while vis.Run():
    time = system.GetChTime()

    
    
    

    if time < 1.0:
        
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_LD)   
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_RD)   

    elif time >= 1.0 and time < 2.0:
        
        turtlebot.SetMotorSpeed(0, robot.TurtleBot_WheelID_LD)
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_RD)

    elif time >= 2.0 and time < 3.0:
        
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_LD)
        turtlebot.SetMotorSpeed(0, robot.TurtleBot_WheelID_RD)

    else:
        
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_LD)
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_RD)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    turtlebot.Update()
    system.DoStepDynamics(time_step)

print("Simulation finished.")