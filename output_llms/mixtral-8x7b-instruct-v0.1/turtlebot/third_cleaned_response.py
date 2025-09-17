import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.irrlicht as chronoirr
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


sensor_manager = sens.ChSensorManager()
system.AddSensorManager(sensor_manager)


lidar_sensor = sens.ChLidarSensor()
lidar_sensor.SetRange(30)
lidar_sensor.SetResolution(0.5)
lidar_sensor.SetAngleMin(-math.pi / 2)
lidar_sensor.SetAngleMax(math.pi / 2)
lidar_sensor.SetPosition(chrono.ChVector3d(0.1, 0, 0.1))
lidar_sensor.SetOrientation(chrono.Q_from_AngX(math.pi / 2))
lidar_sensor.SetRangeFilter(sens.ChLidarSensor.RangeFilter_BOX)
lidar_sensor.SetRangeFilterBox(chrono.ChVector3d(-0.1, -0.1, 0), chrono.ChVector3d(0.1, 0.1, 1))
sensor_manager.Add(lidar_sensor)


for i in range(5):
    box_shape = chrono.ChBox(1, 1, 1)
    box_body = chrono.ChBodyEasyBox(1, 1, 1, 100, True, True)
    box_body.AddVisualShape(box_shape)
    box_body.SetPos(chrono.ChVectorD(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0))
    system.AddBody(box_body)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  


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
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(math.pi, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(0, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(math.pi, 1)


time = 0
while vis.Run():
    move('straight')
    sensor_manager.Update(time_step)

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)