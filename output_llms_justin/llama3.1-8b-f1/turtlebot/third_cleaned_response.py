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


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.2, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(-0.2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.2, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(0.2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-0.2, RIGHT_DRIVE_WHEEL)
    else:
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


sens_manager = sens.ChSensorManager(system)
sens_manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(2, 2, 2), 500)
sens_manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(2, 2, 2), 500)
sens_manager.scene.AddAreaLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(2, 2, 2), 500, chrono.ChVector3f(0,0,1))


offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    robot.GetChBody(),              
    update_rate,                    
    offset_pose,                    
    horizontal_samples,             
    vertical_samples,               
    horizontal_fov,                 
    max_vert_angle,                 
    min_vert_angle,
    100.0,                          
    sens.LidarBeamShape_RECTANGULAR,  
    sample_radius,                  
    divergence_angle,               
    divergence_angle,               
    return_mode                     
)
lidar.PushFilter(sens.ChFilterLidarProcess())
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)
sens_manager.AddSensor(lidar)


offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
camera = sens.ChCameraSensor(
    robot.GetChBody(),              
    update_rate,                    
    offset_pose,                    
    image_width,                    
    image_height,                   
    fov                             
)
camera.PushFilter(sens.ChFilterCameraProcess())
camera.SetName("Camera Sensor")
camera.SetLag(lag)
sens_manager.AddSensor(camera)


gps = sens.ChGPSSensor(robot.GetChBody(), update_rate, 100.0)
gps.PushFilter(sens.ChFilterGPSProcess())
gps.SetName("GPS Sensor")
gps.SetLag(lag)
sens_manager.AddSensor(gps)


imu = sens.ChIMUSensor(robot.GetChBody(), update_rate, 100.0, 9e9)
imu.PushFilter(sens.ChFilterIMUProcess())
imu.SetName("IMU Sensor")
imu.SetLag(lag)
sens_manager.AddSensor(imu)


offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
scanner = sens.ChScannerSensor(
    robot.GetChBody(),              
    update_rate,                    
    offset_pose,                    
    horizontal_samples,             
    vertical_samples,               
    horizontal_fov,                 
    max_vert_angle,                 
    min_vert_angle,
    100.0,                          
    sens.ScannerBeamShape_RECTANGULAR,  
    sample_radius,                  
    divergence_angle,               
    divergence_angle,               
    return_mode                     
)
scanner.PushFilter(sens.ChFilterScannerProcess())
scanner.SetName("Scanner Sensor")
scanner.SetLag(lag)
scanner.SetCollectionWindow(collection_time)
sens_manager.AddSensor(scanner)


def render():
    sens_manager.BeginScene()
    sens_manager.Render()
    sens_manager.EndScene()


def Update():
    time = robot.GetSystem().GetChTime()
    if abs(time - 1.0) < 1e-4:
        lidar.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(1.5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    if abs(time - 2.0) < 1e-4:
        lidar.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(-1.5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    sens_manager.Update()


boxes = []
for i in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box.SetPos(chrono.ChVector3d(0.5 + (i % 2) * 1.0, 0, 1.5 + (i // 2) * 1.0))
    box.SetFixed(False)
    box.SetMass(1000)
    system.Add(box)
    boxes.append(box)


render()
while vis.Run():
    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    sens_manager.Update()

    
    move('straight')

    
    for box in boxes:
        if box.GetPos().y - 0.3 < robot.GetChBody().GetPos().y and box.GetPos().y + 0.3 > robot.GetChBody().GetPos().y and abs(box.GetPos().x - robot.GetChBody().GetPos().x) < 0.5:
            robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
            robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
        if box.GetPos().x + 0.3 > robot.GetChBody().GetPos().x and box.GetPos().x - 0.3 < robot.GetChBody().GetPos().x and box.GetPos().z - 0.3 < robot.GetChBody().GetPos().z and box.GetPos().z + 0.3 > robot.GetChBody().GetPos().z:
            robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
            robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)