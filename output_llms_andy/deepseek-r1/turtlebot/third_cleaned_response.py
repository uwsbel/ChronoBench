import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  


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


box_mat = chrono.ChContactMaterialNSC()
for i in range(5):
    box = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, True, box_mat)
    x = np.random.uniform(-2, 2)
    y = np.random.uniform(-2, 2)
    box.SetPos(chrono.ChVector3d(x, y, 0))
    box.SetFixed(False)
    system.Add(box)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3d(2, 2.5, 5), chrono.ChColor(0.8, 0.8, 0.8), 5000)


lidar_offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.1), 
                                    chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 0, 1)))
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  
    10,                      
    lidar_offset_pose,       
    1800,                    
    24,                      
    360,                     
    30,                      
    100.0,                   
    sens.LidarModel_VELODYNE,
    sens.VisualizationType_NONE
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0.0)
lidar.PushFilter(sens.FilterXYZAccess())


noise_model = sens.NoiseNormalDist(0.0, 0.016)
lidar.SetNoiseModel(noise_model)
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


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, -math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, -math.pi)
    elif mode == 'left':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, 0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, -math.pi)
    elif mode == 'right':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, -math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)


time_step = 2e-3


time = 0
straight_mode = True
right_turn_mode = False

while vis.Run():
    
    if straight_mode:
        move('straight')
    
    
    if time >= 1.0 and straight_mode:
        move('right')
        straight_mode = False
        right_turn_mode = True
    
    
    if time >= 2.0 and right_turn_mode:
        move('straight')
        right_turn_mode = False

    
    time += time_step

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)