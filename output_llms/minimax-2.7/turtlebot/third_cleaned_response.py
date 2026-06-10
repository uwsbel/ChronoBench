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


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  


sensor_manager = sens.ChSensorManager(system)
fov = 3.14159 / 4  
update_rate = 30.0
image_width = 640
image_height = 480


lidar_pose = chrono.ChFrameD(chrono.ChVector3d(0, 0, 0.5))  
lidar = sens.ChLidarSensor(
    robot.GetBody(),
    update_rate,
    lidar_pose,
    fov,
    image_width,
    image_height,
    0.1,   
    30.0  
)
lidar.SetName("Lidar Sensor")
lidar.SetBaseTexture(chrono.GetChronoDataFile("textures/blue.png"))


lidar_noise = sens.ChNoiseGaussParams(0.0, 0.05)
lidar.SetNoise(lidar_noise)


lidar.AddFilter(sens.ChFilterDIAccess())
lidar.AddFilter(sens.ChFilterXYZOutput())
lidar.AddFilter(sens.ChFilterVisualizeImage(image_width, image_height, "Lidar Data"))

sensor_manager.AddSensor(lidar)


box_mat = chrono.ChContactMaterialNSC()
boxes = []
for i in range(5):
    box_size = np.random.uniform(0.2, 0.5, 3)
    box = chrono.ChBodyEasyBox(
        box_size[0], box_size[1], box_size[2],
        1000, True, True, box_mat
    )
    box_pos = chrono.ChVector3d(
        np.random.uniform(-3, 3),
        np.random.uniform(-3, 3),
        np.random.uniform(0, 2)
    )
    box.SetPos(box_pos)
    box.SetFixed(False)
    system.Add(box)
    boxes.append(box)


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
    
    
    if abs(time - 1.0) < 1e-4:
        move('left')
        
    
    if abs(time - 2.0) < 1e-4:
        move('right')
    
    
    time += time_step

    
    sensor_manager.Update()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)