import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1.2, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
try:
    import pychrono.robot as turtlebot
    robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
    robot.Initialize()  
except AttributeError:
    print("TurtleBot class or its methods are not available.")


box_mat = chrono.ChContactMaterialNSC()
for _ in range(5):
    box_pos = chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.5)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 1, 1000, True, True, box_mat)
    box.SetPos(box_pos)
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





manager = sens.ChSensorManager(system)

lidar = sens.ChLidarSensor(
    robot.body, 
    10, 
    chrono.ChFrame(chrono.ChVector3d(0, 0, 0.5), chrono.Q_from_AngAxis(0, chrono.VECT_Z)), 
    100, 
    30, 
    chrono.ChVector3d(0.5, 0.1, 0.01)
)
lidar.SetName("Lidar Sensor")
lidar.PushFilter(sens.ChFilterDIAreaReconstruction())
lidar.PushFilter(sens.ChFilterVisualize(512, 512, "Lidar Output"))
manager.AddSensor(lidar)


def move(mode):
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    speed = math.pi  
    if mode == 'straight':
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


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