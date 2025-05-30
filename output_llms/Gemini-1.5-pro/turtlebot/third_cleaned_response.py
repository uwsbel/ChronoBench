import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)

ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  


manager = sens.ChSensorManager(system)



lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.2)),  
    math.pi / 4.0,  
    
    chrono.CH_C_PI / 12,  
    50,  
    5,  
    20.0,  
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)  
lidar.AddNoise(0.01)  
lidar.SetOffset(0.01, 0.1)  



manager.AddSensor(lidar)



def generate_random_pos(x_range, y_range, z_range):
    x = np.random.uniform(x_range[0], x_range[1])
    y = np.random.uniform(y_range[0], y_range[1])
    z = np.random.uniform(z_range[0], z_range[1])
    return chrono.ChVector3d(x, y, z)


num_boxes = 5
for i in range(num_boxes):
    size = np.random.uniform(0.2, 0.5)
    box = chrono.ChBodyEasyBox(size, size, size, 1000, True, True, ground_mat)
    box.SetPos(generate_random_pos((-5, 5), (-5, 5), (0.5, 1.5)))
    system.Add(box)



def move(mode):
    
    if mode == "straight":
        robot.SetMotorSpeed(math.pi / 2, 0)  
        robot.SetMotorSpeed(math.pi / 2, 1)  
    elif mode == "left":
        robot.SetMotorSpeed(math.pi / 4, 0)
        robot.SetMotorSpeed(-math.pi / 4, 1)
    elif mode == "right":
        robot.SetMotorSpeed(-math.pi / 4, 0)
        robot.SetMotorSpeed(math.pi / 4, 1)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)





time_step = 2e-3


time = 0
while vis.Run():
    move("straight")  

    
    manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)