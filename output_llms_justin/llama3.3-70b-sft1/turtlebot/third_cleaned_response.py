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


offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),
    update_rate,
    offset_pose,
    horizontal_samples,
    vertical_samples,
    horizontal_fov,
    max_vert_angle,
    min_vert_angle,
    100
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)

if vis:
    vis_filter = sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data")
    lidar.PushFilter(vis_filter)
    vis.PushFilter(vis_filter)

if noise_model == "CONST_NORMAL_XYZI":
    noise_filter = sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01, 0.5)
elif noise_model == "NONE":
    noise_filter = sens.ChFilterLidarNoNoise()
elif noise_model == "CONST_NORMAL_XYZ":
    noise_filter = sens.ChFilterLidarNoiseXYZ(0.01, 0.001, 0.001, 0.5)
lidar.PushFilter(noise_filter)

if vis:
    vis_filter = sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Noisy Lidar Depth Data")
    lidar.PushFilter(vis_filter)
    vis.PushFilter(vis_filter)

if pc_format == " Legacy":
    lidar.PushFilter(sens.ChFilterDIAccess())
elif pc_format == "PNG":
    png_filter = sens.ChFilterDIAccessPNG()
    png_filter.SetPath(out_dir + "/lidar_depth")
    lidar.PushFilter(png_filter)
elif pc_format == "CV":
    cv_filter = sens.ChFilterOpenCV()
    cv_filter.SetPath(out_dir + "/lidar_depth")
    lidar.PushFilter(cv_filter)
elif pc_format == "NPY":
    npy_filter = sens.ChFilterDIAccessNPY()
    npy_filter.SetPath(out_dir + "/lidar_depth")
    lidar.PushFilter(npy_filter)

if vis:
    vis_filter = sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Depth Data (m)")
    lidar.PushFilter(vis_filter)
    vis.PushFilter(vis_filter)

if vis:
    lidar.PushFilter(sens.ChFilterDIAccess())
    vis.PushFilter(sens.ChFilterDIAccess())

manager.AddSensor(lidar)


for _ in range(5):
    x = random.uniform(1, 5)
    y = random.uniform(-5, 5)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box.SetPos(chrono.ChVector3d(x, y, 0.25))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
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





time_step = 2e-3


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(2, 0)
        robot.SetMotorSpeed(2, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(2, 0)
        robot.SetMotorSpeed(-2, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(-2, 0)
        robot.SetMotorSpeed(2, 1)


time = 0
while vis.Run():
    
    move('straight')

    
    manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)