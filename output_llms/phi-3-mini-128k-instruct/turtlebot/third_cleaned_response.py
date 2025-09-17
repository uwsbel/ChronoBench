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
lidar_sensor = sens.ChLidarSensor(sensor_manager, system)




lidar_sensor.SetName("Lidar")
lidar_sensor.SetRange(1000)
lidar_sensor.SetMaxRange(5000)
lidar_sensor.SetMinAngle(0)
lidar_sensor.SetMaxAngle(math.pi)
lidar_sensor.SetMinElevation(0)
lidar_sensor.SetMaxElevation(math.pi / 2)
lidar_sensor.SetMinAzimuth(0)
lidar_sensor.SetMaxAzimuth(2 * math.pi)
lidar_sensor.SetMinAngleFilter(0.1)
lidar_sensor.SetMaxAngleFilter(0.1)
lidar_sensor.SetMinElevationFilter(0.1)
lidar_sensor.SetMaxElevationFilter(0.1)
lidar_sensor.SetMinAzimuthFilter(0.1)
lidar_sensor.SetMaxAzimuthFilter(0.1)
system.Add(lidar_sensor)


for _ in range(5):
    box_pos = chrono.ChVector3d(np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(-0.5, -0.1))
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, chrono.ChContactMaterialNSC())
    box.SetPos(box_pos)
    box.SetRot(chrono.ChQuaterniond(math.cos(np.random.uniform(0, 2 * math.pi)), 0, 0, math.sin(np.random.uniform(0, 2 * math.pi))))
    system.Add(box)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with Sensors')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)





time_step = 2e-3


time = 0
while vis.Run():
    
    sensor_manager.Update()

    
    robot.move('straight')

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)