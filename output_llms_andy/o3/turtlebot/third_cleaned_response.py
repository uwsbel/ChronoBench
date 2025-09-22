import os
import math
import random
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens                           
from pychrono import irrlicht as chronoirr




system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)




ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1,          
                              1000,               
                              True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.6))       
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)




init_pos = chrono.ChVectorD(0, 0.2, 0)                                  
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)                             
robot    = turtlebot.TurtleBot(system, init_pos, init_rot)              
robot.Initialize()


LEFT_DRIVE_WHEEL  = 0
RIGHT_DRIVE_WHEEL = 1




box_mat = chrono.ChContactMaterialNSC()
for i in range(5):                                                      
    bx = chrono.ChBodyEasyBox(0.25, 0.25, 0.25, 1000, True, True, box_mat)
    x = random.uniform(-3.0, 3.0)
    y = random.uniform(-3.0, 3.0)
    bx.SetPos(chrono.ChVectorD(x, y, 0.125))                            
    bx.SetMass(5.0)
    bx.SetCollide(True)
    bx.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.1, 0.1))
    system.Add(bx)




manager = sens.ChSensorManager(system)                                  


manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100),
                            chrono.ChVectorF(1, 1, 1), 500)


parent_body = robot.GetBody()                                           


lidar_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.20),
                               chrono.ChQuaternionD(1, 0, 0, 0))


lidar = sens.ChLidarSensor(parent_body,                   
                           10.0,                          
                           lidar_offset,                  
                           32,                            
                           720,                           
                           math.radians(90),              
                           math.radians(360),             
                           80.0)                          


lidar.PushFilter(sens.ChFilterLidarNoise(0.00))
lidar.PushFilter(sens.ChFilterLidarXYZReturn())           

manager.AddSensor(lidar)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot – Lidar demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5),
                       chrono.ChVectorD(0, 0, 0.5),
                       3, 4, 10, 40, 512)
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.7),
              chrono.ChVectorD(0, 0, 0.3))

vis.SetCameraVertical(chrono.CameraVerticalDir_Z)




def move(mode='straight', speed=math.pi):
    
    if mode == 'straight':
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0.0,  LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0,  RIGHT_DRIVE_WHEEL)
    else:     
        robot.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)




time_step = 2e-3
time      = 0.0

while vis.Run():
    
    move('straight')                       

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)
    manager.Update()                       
    time += time_step