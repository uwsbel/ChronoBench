import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.6))  
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()


sensor_mgr = sens.ChSensorManager(system)
system.AddSensorManager(sensor_mgr)


lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetParentFrame(robot.GetChassisBody().GetFrame_REF_to_abs())  
lidar.SetPosition(chrono.ChVectorD(0, 0, 0.2))  
lidar_params = sens.ChLidarParameters()
lidar_params.m_fov_horizontal = 270  
lidar_params.m_fov_vertical = 30
lidar_params.m_range = 10.0  
lidar_params.m_resolution_horiz = 0.5  
lidar_params.m_resolution_vert = 1.0
lidar_params.m_noise = 0.01
lidar.SetParameters(lidar_params)


range_filter = sens.ChFilterRange()
range_filter.SetMinDistance(0.1)
range_filter.SetMaxDistance(10.0)
lidar.AddFilter(range_filter)

point_cloud_filter = sens.ChFilterPointCloud()
lidar.AddFilter(point_cloud_filter)

sensor_mgr.AddSensor(lidar)


for _ in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    z = 0.15  
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1, True, True, ground_mat)
    box.SetPos(chrono.ChVectorD(x, y, z))
    box.SetBodyFixed(False)
    system.Add(box)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with Sensors')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

def move(robot, mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


time_step = 2e-3
time = 0
while vis.Run():
    
    move(robot, 'straight')

    
    system.DoStepDynamics(time_step)

    
    sensor_mgr.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    time += time_step

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.6))  
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()


sensor_mgr = sens.ChSensorManager(system)
system.AddSensorManager(sensor_mgr)


lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetParentFrame(robot.GetBody().GetFrame_REF_to_abs())  
lidar.SetPosition(chrono.ChVectorD(0, 0, 0.2))  

lidar_params = sens.ChLidarParameters()
lidar_params.m_fov_horizontal = 270  
lidar_params.m_fov_vertical = 30
lidar_params.m_range = 10.0  
lidar_params.m_resolution_horiz = 0.5  
lidar_params.m_resolution_vert = 1.0
lidar_params.m_noise = 0.01
lidar.SetParameters(lidar_params)


range_filter = sens.ChFilterRange()
range_filter.SetMinDistance(0.1)
range_filter.SetMaxDistance(10.0)
lidar.AddFilter(range_filter)

point_cloud_filter = sens.ChFilterPointCloud()
lidar.AddFilter(point_cloud_filter)

sensor_mgr.AddSensor(lidar)


for _ in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    z = 0.15  
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1, True, True, ground_mat)
    box.SetPos(chrono.ChVectorD(x, y, z))
    box.SetBodyFixed(False)
    system.Add(box)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with Sensors')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

def move(robot, mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


time_step = 2e-3
time = 0
while vis.Run():
    
    move(robot, 'straight')

    
    system.DoStepDynamics(time_step)

    
    sensor_mgr.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    time += time_step