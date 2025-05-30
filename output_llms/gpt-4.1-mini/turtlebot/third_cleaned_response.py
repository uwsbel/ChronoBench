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


ground_mat = chrono.ChMaterialSurfaceNSC()  
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


np.random.seed(42)  
for i in range(5):
    box_size = [0.3, 0.3, 0.3]
    box_pos = chrono.ChVector3d(np.random.uniform(-5, 5),
                                np.random.uniform(-5, 5),
                                box_size[2] / 2 - 0.6)  
    box_mat = chrono.ChMaterialSurfaceNSC()
    box = chrono.ChBodyEasyBox(box_size[0], box_size[1], box_size[2], 1000, True, True, box_mat)
    box.SetPos(box_pos)
    box.SetMass(5)
    box.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    box.SetCollide(True)
    system.Add(box)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()



manager = sens.ChSensorManager(system)
manager.SetVerbose(False)
manager.Initialize()


sensor_update_rate = 30  



lidar_pose = chrono.ChFrameD()
lidar_pose.SetPos(chrono.ChVectorD(0, 0, 0.15))


lidar = sens.ChLidarSensor(
    robot.GetBody(),          
    sensor_update_rate,       
    lidar_pose,               
    25.0,                    
    120,                     
    math.radians(40),        
    3,                       
    math.radians(20),        
    20,                      
    20000                    
)



filter_xyz = sens.ChFilterPCfromXYZI()
lidar.PushFilter(filter_xyz)


filter_indigo = sens.ChFilterPCtoColorIndigo(0, 20)
filter_xyz.PushFilter(filter_indigo)


filter_vis = sens.ChFilterVisualizePointCloud()
filter_indigo.PushFilter(filter_vis)
vis.AddVisualModel(filter_vis.GetVisualModel())


manager.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with sensors')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)





time_step = 2e-3


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


def move(mode):
    if mode == 'straight':
        
        speed = math.pi / 2  
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi / 2, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        
        robot.SetMotorSpeed(math.pi / 2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    elif mode == 'stop':
        
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


time = 0.0
while vis.Run():
    
    move('straight')

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    time += time_step