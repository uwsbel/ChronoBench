import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr


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


for i in range(5):
    box = chrono.ChBodyEasyBox(1, 1, 1.5, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.75))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    system.Add(box)

init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  


manager = sens.ChSensorManager(system)


offset_pose = chrono.ChFramed(
        chrono.ChVector3d(5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )


update_rate = 5.0

horizontal_samples = 800
vertical_samples = 300

horizontal_fov = 2 * chrono.CH_PI  
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6

lag = 0

collection_time = 1. / update_rate  

sample_radius = 2

divergence_angle = 0.003

return_mode = sens.LidarReturnMode_STRONGEST_RETURN
lidar = sens.ChLidarSensor(
    robot.GetChassis().GetBody(),              
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
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
manager.AddSensor(lidar)

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
    
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    if mode == 'straight':
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi/2, LEFT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi/2, RIGHT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
    else:
        raise ValueError('Invalid mode')

time = 0
while vis.Run():
    move('straight')
    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    manager.Update()
    
    system.DoStepDynamics(time_step)