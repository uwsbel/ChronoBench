import os
import math
import numpy as np

import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr




system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)




ground = chrono.ChBodyEasyBox(20, 20, 1,    
                              1000,         
                              True, True)   
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetBodyFixed(True)

ground_mat = ground.GetMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)

system.Add(ground)




box = chrono.ChBodyEasyBox(0.25, 5, 0.25,
                          1000,
                          True, True)
box.SetPos(chrono.ChVector3d(0, 0, 0.0))
box.SetBodyFixed(True)
box_mat = box.GetMaterialSurfaceNSC()
box_mat.SetFriction(0.9)
system.Add(box)




rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(-5, 0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

rover.Initialize(chrono.ChFrameD(init_pos, init_rot))




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain with LiDAR')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 3),
              chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0, 0, 0),
                       3, 4, 10, 40, 512)





manager = sens.ChSensorManager(system)
manager.SetVerbose(False)





update_rate     = 10              
horiz_samples   = 360             
vert_samples    = 1               
horiz_fov       = chrono.CH_C_2PI 
vert_fov        = chrono.CH_C_PI/8   
min_distance    = 0.5             
max_distance    = 20.0            


lidar = sens.ChLidarSensor(rover.GetChassisBody(),
                           update_rate,
                           horiz_samples,
                           vert_samples,
                           horiz_fov,
                           vert_fov,
                           min_distance,
                           max_distance)








vis_filter = sens.ChFilterVisualizePointCloud(max_points=3000)
lidar.PushFilter(vis_filter)


pcd_folder = os.path.join(os.getcwd(), "lidar_scans")
os.makedirs(pcd_folder, exist_ok=True)
pcd_filter = sens.ChFilterPCDWriter(
    output_folder=pcd_folder,
    filename_format="scan_{:05d}.pcd",
    write_every=1)   
lidar.PushFilter(pcd_filter)


manager.AddSensor(lidar)




time_step = 1e-3
while vis.Run():
    
    driver.SetSteering(0.0)

    
    rover.Update()

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)