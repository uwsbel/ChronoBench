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


np.random.seed(42)
box_mat = chrono.ChContactMaterialNSC()
for i in range(5):
    box_x = float(np.random.uniform(-4, 4))
    box_y = float(np.random.uniform(-4, 4))
    box = chrono.ChBodyEasyBox(0.4, 0.4, 0.4, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(box_x, box_y, 0.0))
    box.SetFixed(True)
    system.Add(box)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


def move(mode):
    
    if mode == 'straight':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)




sensor_manager = sens.ChSensorManager(system)
sensor_manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(1, 1, 1),
    5000
)


lidar_update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * math.pi          
max_vert_angle = math.pi / 12         
min_vert_angle = -math.pi / 6         
max_range = 100.0                      
lag = 0.0
collection_time = 1.0 / lidar_update_rate


lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)


lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),       
    lidar_update_rate,            
    lidar_offset_pose,            
    horizontal_samples,           
    vertical_samples,             
    horizontal_fov,               
    max_vert_angle,               
    min_vert_angle,               
    max_range,                    
    sens.LidarBeamShape_RECTANGULAR,  
    2,                            
    0.003,                        
    0.003,                        
    sens.LidarReturnMode_STRONGEST_RETURN  
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)


lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterXYZIToBuf())
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Point Cloud"))


sensor_manager.AddSensor(lidar)




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
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512
)


time_step = 2e-3




time = 0
while vis.Run():
    
    move('straight')

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sensor_manager.Update()

    
    system.DoStepDynamics(time_step)

    
    time += time_step