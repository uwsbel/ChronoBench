import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
from pychrono import sensor as sens


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
system.Add(sensor_manager)


lidar = sens.ChLidarSensor(robot.GetChassisBody(),  
                           10,                       
                           chrono.ChFrameD(chrono.ChVector3d(0.1, 0, 0.1), chrono.QUNIT),  
                           1.0,                      
                           0.1,                      
                           0.01,                     
                           360,                      
                           -30,                      
                           30)                       


noise_model = sens.ChGaussianNoiseModel(0.02, 0.001)
lidar.AddNoiseModel(noise_model)
lidar.AddFilter(sens.ChFilterAddOutlier(0.1))
lidar.AddFilter(sens.ChFilterAddGaussian(0.01))

sensor_manager.AddSensor(lidar)


np.random.seed(42)  
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.3)
box_mat.SetRestitution(0.2)

for i in range(5):
    box = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 10, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-2, 2),
                                np.random.uniform(-2, 2),
                                np.random.uniform(0.1, 0.5)))
    box.SetRot(chrono.ChQuaterniond(np.random.uniform(0, 1),
                                   np.random.uniform(0, 1),
                                   np.random.uniform(0, 1),
                                   np.random.uniform(0, 1)))
    box.GetVisualShape(0).SetColor(chrono.ChColor(np.random.uniform(0, 1),
                                                 np.random.uniform(0, 1),
                                                 np.random.uniform(0, 1)))
    system.Add(box)


def move(mode):
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1

    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
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

    
    sensor_manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)