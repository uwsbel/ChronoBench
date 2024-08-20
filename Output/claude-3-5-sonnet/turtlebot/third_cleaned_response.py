import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.6))  
ground.SetBodyFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 20, 20)
system.Add(ground)


init_pos = chrono.ChVectorD(0, 0, 0.2)  
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(0, 0, 100), chrono.ChVectorF(1, 1, 1), 500)


offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0))
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  
    20,                      
    offset_pose,             
    1000,                    
    50,                      
    chrono.CH_C_PI,          
    chrono.CH_C_PI / 6.,     
    100.0                    
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)


lidar.PushFilter(sens.ChFilterLidarNoiseSimple(0.01, 0.001))

lidar.PushFilter(sens.ChFilterVisualize(1000, "Lidar Data"))

manager.AddSensor(lidar)


box_material = chrono.ChMaterialSurfaceNSC()
for _ in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 100, True, True, box_material)
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    box.SetPos(chrono.ChVectorD(x, y, 0))
    system.Add(box)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)


time_step = 2e-3


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)


time = 0
while vis.Run():
    
    move('straight')

    
    time += time_step

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)