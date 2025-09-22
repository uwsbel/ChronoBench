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


sensor_manager = sens.ChSensorManager(system)
sensor_manager.Initialize()

lidar = sens.ChSensorLidar()
lidar.SetName("LIDAR")
lidar.SetParent(robot.GetChassisBody())
lidar.SetPose(chrono.ChFrame(chrono.ChVector3d(0.2, 0, 0.2), chrono.ChQuaterniond(1, 0, 0, 0)))
lidar.SetBeamCount(360)
lidar.SetBeamRange(5)
lidar.SetBeamResolution(0.5)
lidar.SetScanRate(10)
lidar.SetBeamThickness(0.01)
lidar.SetBeamColor(chrono.ChColor(1, 0, 0))
lidar.SetPointColor(chrono.ChColor(0, 1, 0))

lidar.AddFilter(sens.ChFilterLidarRange(0.1, 5))
lidar.AddFilter(sens.ChFilterLidarAngle(-math.pi, math.pi))
lidar.AddFilter(sens.ChFilterLidarIntensity(0.5))

sensor_manager.AddSensor(lidar)
sensor_manager.AddVisualizationPointCloud(lidar, 0.01, chrono.ChColor(0, 1, 0))


np.random.seed(0)  
for i in range(5):
    size = 0.5 + np.random.rand() * 0.5  
    pos = chrono.ChVector3d(np.random.rand() * 4 - 2, np.random.rand() * 4 - 2, 0.25 + size / 2)
    box = chrono.ChBodyEasyBox(size, size, size, 1000, True, True, ground_mat)
    box.SetPos(pos)
    box.SetFixed(False)
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
        robot.SetMotorSpeed(1, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(1, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(1, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-1, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-1, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(1, RIGHT_DRIVE_WHEEL)
    else:
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


time = 0
while vis.Run():
    
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1

    
    move('straight')

    
    sensor_manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)