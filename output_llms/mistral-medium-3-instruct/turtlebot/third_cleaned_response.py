import os
import math
import random
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
from pychrono import sensor as sens


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


sensor_manager = sens.ChSensorManager(system)
system.Add(sensor_manager)


lidar = sens.ChLidarSensor(robot.GetChassisBody(),  
                           10,                      
                           chrono.ChFrameD(chrono.ChVector3d(0.1, 0, 0.1),  
                                           chrono.Q_from_AngAxis(-chrono.CH_PI/4, chrono.ChVector3d(0, 0, 1))),  
                           1.0,                     
                           0.5,                     
                           1.0,                     
                           0.5,                     
                           0.1,                     
                           5.0)                     
lidar.SetName("LidarSensor")
lidar.AddFilter(sens.ChFilterRayC2C())  
lidar.AddFilter(sens.ChFilterPC2Grid()) 
sensor_manager.AddSensor(lidar)


box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.3)
box_mat.SetRestitution(0.1)

for i in range(5):
    box = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(random.uniform(-2, 2), random.uniform(-2, 2), 0.1))
    box.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
    box.GetVisualShape(0).SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
    system.Add(box)


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, 0)  
        robot.SetMotorSpeed(math.pi, 1)  
    elif mode == 'left':
        robot.SetMotorSpeed(0, 0)        
        robot.SetMotorSpeed(-math.pi, 1) 
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, 0) 
        robot.SetMotorSpeed(0, 1)        


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with Lidar')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)





time_step = 2e-3


time = 0
while vis.Run():
    
    move('straight')

    
    sensor_manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)