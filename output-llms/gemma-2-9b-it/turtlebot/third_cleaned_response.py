import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
from pychrono.sensor import *


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


sensor_manager = SensManager(system)


lidar = SensLidar(
    sensor_manager,
    name="lidar",
    position=chrono.ChVector3d(0, 0.1, 0.2),
    orientation=chrono.ChQuaterniond(1, 0, 0, 0),
    range=5.0,
    angle_min=-math.pi / 2,
    angle_max=math.pi / 2,
    angle_step=math.pi / 180,
    filter_type=SensLidar.FilterType.GAUSSIAN,
    filter_sigma=0.1,
)
sensor_manager.AddSensor(lidar)


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
    
    move('straight')

    
    sensor_manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.5, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.5, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0.5, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-0.5, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-0.5, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.5, RIGHT_DRIVE_WHEEL)
    else:
        print("Invalid mode!")




for i in range(5):
    box_size = chrono.ChVector3d(0.5, 0.5, 0.5)
    box_pos = chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.1)
    box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 1000, True, True)
    box.SetPos(box_pos)
    box.SetFixed(False)
    system.Add(box)