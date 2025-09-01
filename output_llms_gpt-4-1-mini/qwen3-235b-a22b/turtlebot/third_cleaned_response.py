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


init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()


for i in range(5):
    box_size = 0.4
    box_mat = chrono.ChContactMaterialNSC()
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000, True, True, box_mat)
    x = np.random.uniform(-10, 10)
    y = np.random.uniform(-10, 10)
    z_pos = -0.1 + box_size / 2  
    box.SetPos(chrono.ChVector3d(x, y, z_pos))
    box.SetFixed(False)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    system.Add(box)


manager = sens.ChSensorManager(system)
manager.SetVerbose(False)


lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.2),
    chrono.ChQuaterniond(1, 0, 0, 0)
)
lidar = sens.ChLidarSensor(
    robot.GetBody(),  
    10,  
    lidar_offset,
    100,  
    1,    
    chrono.CH_C_PI,  
    chrono.CH_C_PI / 180 * 10,  
    100.0  
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)


lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud("Lidar Point Cloud"))
manager.AddSensor(lidar)


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(-2 * math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-2 * math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-2 * math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-2 * math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


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
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)
    time += time_step
    
    
    manager.Update()