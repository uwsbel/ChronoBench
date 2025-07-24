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
    
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    
    if abs(time - 1.0) < 1e-4:
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
        
    
    if abs(time - 2.0) < 1e-4:
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    sens_manager.Update()

    
    robot.move('straight')





manager = sens.ChSensorManager(ground)
manager.scene.AddLight(chrono.ChVector3f(2, 2.5, 100).xyzla(), 1000000, 100)





lidar = sens.ChLidarSensor(
    manager,
    100,  
    300.,  
    0.01,  
    1.408,  
    100,    
    chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
    True,  
    sens.LidarBeamShape_RECTANGULAR
)
lidar.PushFilter(sens.ChFilterLidarProcessRaw(sens.FilterOutlierByDistance(2, 10)))
manager.AddSensor(lidar)





camera = sens.ChCameraSensor(
    manager,
    30,  
    1.408,  
    1000.,  
    chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
    True,  
)
camera.PushFilter(sens.ChFilterCameraProcessRaw(sens.FilterCompressJPEG(quality=5)))
manager.AddSensor(camera)





pc_sensor = sens.ChPointCloudSensor(
    manager,
    10,  
    300.,  
    0.01,  
    1.408,  
    100,    
    chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
    0.1,  
    True,  
    sens.LidarBeamShape_RECTANGULAR
)
pc_sensor.PushFilter(sens.ChFilterPointCloudProcessRaw(sens.FilterResize(128, 128)))
manager.AddSensor(pc_sensor)





for i in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-10, 10), np.random.uniform(-10, 10), 1))
    box.SetFixed(False)
    box.SetMass(1000)
    system.Add(box)
    motor = chrono.ChLinkMotorRotationAngle()
    motor.Initialize(box, robot.GetChBody(), 0, 0)
    system.Add(motor)