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


sensor_manager = sens.ChSensorManager()
lidar_sensor = sens.ChLidarSensor()
lidar_sensor.SetRange(5)
lidar_sensor.SetFov(math.pi)
lidar_sensor.SetResolution(0.0174533)
lidar_sensor.SetPosition(chrono.ChVector3d(0, 0.2, 0))
lidar_sensor.SetDirection(chrono.ChVector3d(0, 0, -1))
lidar_sensor.SetNumBeams(1024)
lidar_sensor.SetFilter(sens.ChLidarSensor.FilterType.BOX)
lidar_sensor.SetFilterParams(chrono.ChVector3d(0.1, 0.1, 0.1))
sensor_manager.Add(lidar_sensor)
system.Add(sensor_manager)


box_positions = np.random.rand(5, 3) * 10 - 5
boxes = []
for pos in box_positions:
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 100, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(pos[0], pos[1], 0.25))
    boxes.append(box)
    system.Add(box)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with boxes')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


time_step = 2e-3


time = 0
while vis.Run():
    
    robot.move('straight')

    
    sensor_manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)


def move(self, mode):
    if mode == 'straight':
        self.SetMotorSpeed(0.5, LEFT_DRIVE_WHEEL)
        self.SetMotorSpeed(0.5, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        self.SetMotorSpeed(0.5, LEFT_DRIVE_WHEEL)
        self.SetMotorSpeed(-0.5, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        self.SetMotorSpeed(-0.5, LEFT_DRIVE_WHEEL)
        self.SetMotorSpeed(0.5, RIGHT_DRIVE_WHEEL)