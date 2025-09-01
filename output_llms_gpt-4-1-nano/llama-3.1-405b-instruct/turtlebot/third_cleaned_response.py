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


manager = sens.ChSensorManager(system)
system.Add(manager)


lidar = sens.ChLidarSensor(
    system,
    100,  
    chrono.ChFrameD(chrono.ChVector3d(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)),  
    0.1,  
    0.05,  
    0.01,  
    0.1,  
    0.1  
)
manager.AddSensor(lidar)


lidar.AddFilter(sens.ChFilterShadows())
lidar.AddFilter(sens.ChFilterNoise(0.05))


for _ in range(5):
    box_mat = chrono.ChContactMaterialNSC()
    box = chrono.ChBodyEasyBox(
        np.random.uniform(0.5, 2),  
        np.random.uniform(0.5, 2),  
        np.random.uniform(0.5, 2),  
        100,  
        True,  
        True,  
        box_mat
    )
    box.SetPos(chrono.ChVector3d(
        np.random.uniform(-5, 5),
        np.random.uniform(-5, 5),
        np.random.uniform(0, 2)
    ))
    system.Add(box)


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(2, 0)
        robot.SetMotorSpeed(2, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, 0)
        robot.SetMotorSpeed(math.pi, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(-math.pi, 1)


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

    
    manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)