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
lidar_sensor.SetRange(10)
lidar_sensor.SetResolution(10)
lidar_sensor.SetAngle(math.pi / 2)
lidar_sensor.SetPosition(chrono.ChVector3d(0.2, 0, 0.1))
lidar_sensor.SetOrientation(chrono.Q_from_AngX(math.pi / 2))
lidar_sensor.SetFrame(robot.GetFrame(turtlebot.TURTLEBOT_LINK_BASE_LIDAR))
lidar_sensor.SetDataFile("lidar_data.txt")
lidar_sensor.SetFilters([sens.ChLidarSensor.FILTER_MEAN, sens.ChLidarSensor.FILTER_MEDIAN, sens.ChLidarSensor.FILTER_GAUSSIAN])
sensor_manager.Add(lidar_sensor)


for i in range(5):
    box_x = np.random.uniform(-5, 5)
    box_y = np.random.uniform(-5, 5)
    box_z = 0.05
    box_shape = chrono.ChBoxShape(chrono.ChVector3d(0.1, 0.1, 0.1))
    box_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1, True, True)
    box_body.SetPos(chrono.ChVector3d(box_x, box_y, 0))
    box_body.SetMass(1)
    box_body.AddVisualShape(box_shape)
    box_body.AddCollisionShape(box_shape)
    system.Add(box_body)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  

def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0, turtlebot.TURTLEBOT_MOTOR_LEFT)
        robot.SetMotorSpeed(0, turtlebot.TURTLEBOT_MOTOR_RIGHT)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, turtlebot.TURTLEBOT_MOTOR_LEFT)
        robot.SetMotorSpeed(0, turtlebot.TURTLEBOT_MOTOR_RIGHT)
    elif mode == 'right':
        robot.SetMotorSpeed(0, turtlebot.TURTLEBOT_MOTOR_LEFT)
        robot.SetMotorSpeed(-math.pi, turtlebot.TURTLEBOT_MOTOR_RIGHT)


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