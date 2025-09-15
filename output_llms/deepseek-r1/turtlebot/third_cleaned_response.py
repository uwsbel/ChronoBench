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


for i in range(5):
    box_mat = chrono.ChContactMaterialNSC()
    box = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, True, box_mat)
    while True:  
        x = np.random.uniform(-2, 2)
        y = np.random.uniform(-2, 2)
        if math.sqrt(x**2 + (y-0.2)**2) > 0.5:  
            break
    box.SetPos(chrono.ChVector3d(x, y, 0))
    box.SetFixed(False)
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.2, 0.2))
    system.Add(box)


init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()


LEFT_DRIVE_WHEEL = turtlebot.TurtleBot.LEFT
RIGHT_DRIVE_WHEEL = turtlebot.TurtleBot.RIGHT


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
    elif mode == 'stop':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(1, 1, 1), 5000)


lidar_offset = chrono.ChFrameD(chrono.ChVector3d(0, 0, 0.2), chrono.QUNIT)
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),   
    10,                       
    lidar_offset,             
    4500,                     
    32,                       
    2 * math.pi,              
    math.pi/3,                
    math.pi/6,                
    -math.pi/6,               
    100.0                     
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)


lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
manager.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot with Lidar')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)


time_step = 2e-3


time = 0
while vis.Run():
    
    if time < 1.0:
        move('straight')
    elif time < 2.0:
        move('left')
    elif time < 3.0:
        move('right')
    else:
        move('stop')
    
    
    manager.Update()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)
    time += time_step