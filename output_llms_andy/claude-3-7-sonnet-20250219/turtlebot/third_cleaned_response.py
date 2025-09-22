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
system.SetGravitationalAcceleration(chrono.ChVector(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector(0, 0, -0.6))  
ground.SetBodyFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVector(0, 0.2, 0)  
init_rot = chrono.ChQuaternion(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  


box_mat = chrono.ChMaterialSurfaceNSC()
for i in range(5):
    
    rand_x = random.uniform(-5, 5)
    rand_y = random.uniform(-5, 5)
    
    
    box = chrono.ChBodyEasyBox(0.4, 0.4, 0.4, 100, True, True, box_mat)
    box.SetPos(chrono.ChVector(rand_x, rand_y, 0))
    box.SetBodyFixed(False)  
    
    
    system.Add(box)


sens_manager = sens.ChSensorManager(system)
sens_manager.scene.AddPointLight(chrono.ChVector(100, 100, 100), chrono.ChColor(1, 1, 1), 500.0)


lidar_noise = sens.ChNoiseNone() 
lidar_offset_pose = chrono.ChFrame(chrono.ChVector(0, 0, 0.3), chrono.ChQuaternion(1, 0, 0, 0))
intensity_mode = True
horizontal_samples = 1000
vertical_samples = 16
horizontal_fov = 2 * math.pi
vertical_fov = math.pi / 6
max_vert_angle = vertical_fov / 2
min_vert_angle = -vertical_fov / 2
max_distance = 100
min_distance = 0.1
lidar_update_rate = 10

lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),                 
    lidar_update_rate,                       
    lidar_offset_pose,                       
    horizontal_samples,                      
    vertical_samples,                        
    horizontal_fov,                          
    min_vert_angle,                          
    max_vert_angle,                          
    max_distance,                            
    intensity_mode                           
)


lidar.PushFilter(lidar_noise)

lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Data"))


sens_manager.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector(0, 1.5, 0.2), chrono.ChVector(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVectorD(1.5, -2.5, 5.5), 3, chrono.ChColor(0.7, 0.7, 0.7))


time_step = 2e-3


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


time = 0
while vis.Run():
    
    if abs(time - 1.0) < 1e-4:
        move('left')
        
    
    elif abs(time - 2.0) < 1e-4:
        move('right')
    
    
    elif abs(time - 3.0) < 1e-4:
        move('straight')
    
    
    elif time < 1.0:
        move('straight')

    
    time += time_step

    
    sens_manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)