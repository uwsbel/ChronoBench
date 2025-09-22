import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


ground = chrono.ChBodyEasyBox(5, 5, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.05))
ground.SetBodyFixed(True)
system.Add(ground)


texture = chrono.ChTexture()
texture.SetTextureFilename(chrono.GetChronoDataFile("concrete.jpg"))
ground.AddAsset(texture)



turtlebot_pos = chrono.ChVectorD(0, 0, 0.1)  
turtlebot_yaw = math.radians(0)  
turtlebot_rot = chrono.Q_from_AngZ(turtlebot_yaw)


turtlebot = robot.Turtlebot()
turtlebot.Initialize(system, turtlebot_pos, turtlebot_rot)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Turtlebot Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 0.7), chrono.ChVectorD(0, 0, 0.1))
vis.AddTypicalLights()


timestep = 0.01
end_time = 10.0


left_speed = 0.0
right_speed = 0.0
max_speed = 5.0  


time = 0.0
while vis.Run() and time < end_time:
    
    cycle = int(time // 2) % 3
    if cycle == 0:
        
        left_speed = 1.0
        right_speed = 3.0
    elif cycle == 1:
        
        left_speed = 2.0
        right_speed = 2.0
    else:
        
        left_speed = 3.0
        right_speed = 1.0

    
    left_speed = max(-max_speed, min(max_speed, left_speed))
    right_speed = max(-max_speed, min(max_speed, right_speed))

    
    turtlebot.SetMotorSpeeds(left_speed, right_speed)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(timestep)
    time += timestep

vis.GetDevice().closeDevice()