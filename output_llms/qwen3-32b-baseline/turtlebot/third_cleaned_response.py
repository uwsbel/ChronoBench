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


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


sensor_manager = sens.ChSensorManager(system)


lidar_offset = chrono.ChFrameD(chrono.ChVector3d(0.2, 0, 0.1), chrono.ChQuaterniond(1, 0, 0, 0))
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  
    5.0,  
    lidar_offset,
    400,  
    300,  
    2 * math.pi,  
    math.pi / 2,  
    10.0,  
    0.01  
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / 5.0)


noise_model = sens.ChNoiseModelNormal(0.0, 0.01)
lidar.SetNoiseModel(noise_model)


lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualize(500))
lidar.PushFilter(sens.ChFilterSaveData("lidar_data"))

sensor_manager.AddSensor(lidar)


for _ in range(5):
    
    while True:
        x = np.random.uniform(-10, 10)
        y = np.random.uniform(-10, 10)
        z = np.random.uniform(-0.5, 0.5)
        if (x**2 + (y - 0.2)**2 + z**2) > 1.0:
            break
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(x, y, z))
    system.Add(box)


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
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


time = 0
while vis.Run():
    
    move('straight')
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sensor_manager.Update()

    
    system.DoStepDynamics(time_step)
    
    
    time += time_step