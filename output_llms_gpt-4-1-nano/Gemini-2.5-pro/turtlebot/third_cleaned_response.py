import os
import math
import random 
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens 



LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


ROBOT_MAX_SPEED = math.pi


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


num_boxes = 5
box_contact_mat = chrono.ChContactMaterialNSC() 



ground_thickness = 1.0 
ground_top_z = ground.GetPos().z + ground_thickness / 2.0

for i in range(num_boxes):
    size_x = random.uniform(0.2, 0.5)
    size_y = random.uniform(0.2, 0.5)
    size_z = random.uniform(0.2, 0.5)
    
    
    pos_x = random.uniform(-5, 5)
    pos_y = random.uniform(-5, 5)
    pos_z = ground_top_z + size_z / 2.0
    
    box = chrono.ChBodyEasyBox(size_x, size_y, size_z, 1000, True, True, box_contact_mat)
    box.SetPos(chrono.ChVector3d(pos_x, pos_y, pos_z))
    box.GetVisualShape(0).SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
    system.Add(box)


def move(mode):
    
    if mode == 'straight':
        robot.SetMotorSpeedSync(ROBOT_MAX_SPEED, ROBOT_MAX_SPEED)
    elif mode == 'left':
        
        robot.SetMotorSpeedSync(ROBOT_MAX_SPEED * 0.5, ROBOT_MAX_SPEED)
        
    elif mode == 'right':
        
        robot.SetMotorSpeedSync(ROBOT_MAX_SPEED, ROBOT_MAX_SPEED * 0.5)
        
    elif mode == 'stop':
        robot.SetMotorSpeedSync(0, 0)
    else:
        print(f"Unknown move mode: {mode}")
        robot.SetMotorSpeedSync(0, 0)




manager = sens.ChSensorManager(system)


sensor_update_rate = 10.0  




lidar_offset_pose = chrono.ChFrameD(chrono.ChVector3d(0, 0, 0.15), chrono.QUNIT)


lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  
    sensor_update_rate,      
    lidar_offset_pose,       
    360,                     
    1,                       
    2.0 * math.pi,           
    0.02,                    
    10.0                     
)
lidar.SetName("LidarSensor")
lidar.SetLag(0.0) 
lidar.SetMaxPollTime(0.001) 



lidar.PushFilter(sens.ChFilterDIAccess())

lidar.PushFilter(sens.ChFilterPCfromDepth())

manager.AddSensor(lidar)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot with Lidar and Obstacles')
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

    
    
    
    
        
    
    
    

    
    system.DoStepDynamics(time_step)

    
    time += time_step

    
    manager.Update()

    
    vis.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3)) 
    vis.Render()
    
    
    vis.EndScene()