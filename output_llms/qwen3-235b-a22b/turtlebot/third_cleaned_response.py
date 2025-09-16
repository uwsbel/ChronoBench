import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens  
from pychrono import irrlicht as chronoirr


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


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


np.random.seed(42)  
num_boxes = 5
box_half_size = 0.25  
box_material = chrono.ChContactMaterialNSC()
for i in range(num_boxes):
    
    x = np.random.uniform(-10, 10)
    y = np.random.uniform(-10, 10)
    
    z_pos = -0.1 + box_half_size  
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, box_material)
    box.SetPos(chrono.ChVector3d(x, y, z_pos))
    box.SetFixed(False)
    system.Add(box)


manager = sens.ChSensorManager(system)
manager.SetVerbose(0)  


lidar = sens.ChLidarSensor(
    robot,              
    10,                 
    360,                
    1,                  
    chrono.CH_PI,       
    chrono.CH_PI / 12,  
    100.0,              
    sens.LidarBeamShape_RECTANGULAR,  
    2,                  
    0.003,              
    0.003               
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0.1)  
lidar.SetFilter(sens.ChLidarFilterNoiseNone())  
lidar.PushFilter(sens.ChLidarFilterVisualize())  
manager.AddSensor(lidar)


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


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(chrono.CH_PI, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(chrono.CH_PI, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-chrono.CH_PI, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-chrono.CH_PI, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


time = 0
while vis.Run():
    
    move('straight')  

    
    system.DoStepDynamics(time_step)

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    time += time_step