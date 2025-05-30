import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens


system = chrono.ChSystemNSC()
if system is None:
    raise Exception("Failed to create Chrono system")

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
if ground is None:
    raise Exception("Failed to create ground body")

ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
if robot is None:
    raise Exception("Failed to create Turtlebot instance")

robot.Initialize()  


sensor_manager = sens.ChSensorManager(system)
if sensor_manager is None:
    raise Exception("Failed to create sensor manager")


lidar_sensor = sens.ChLidarSensor(chrono.ChFrame3d(chrono.ChVector3d(0.2, 0, 0.5)))
lidar_sensor.Setup(num_horizontal_channels=360, num_vertical_channels=10, horizontal_min_angle=-math.pi, horizontal_max_angle=math.pi, vertical_min_angle=-math.pi/4, vertical_max_angle=math.pi/4, min_distance=0.1, max_distance=10.0)
lidar_sensor.SetUpdateRate(10)
sensor_manager.AddSensor(lidar_sensor)


for _ in range(5):
    box_body = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box_body.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.5))
    system.Add(box_body)


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.5, 0)  
        robot.SetMotorSpeed(0.5, 1)  
    elif mode == 'left':
        robot.SetMotorSpeed(-0.5, 0)  
        robot.SetMotorSpeed(0.5, 1)  
    elif mode == 'right':
        robot.SetMotorSpeed(0.5, 0)  
        robot.SetMotorSpeed(-0.5, 1)  


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
    try:
        move('straight')  

        
        sensor_manager.Update()

        
        time += time_step

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        system.DoStepDynamics(time_step)
    except Exception as e:
        print(f"Error during simulation: {e}")
        break