import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.sensor as sens

import numpy as np
import random
import math

# Create the Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Set the gravitational force for the system
system.SetGravitationalAcceleration(chrono.ChVector3d(0,0,0))

# Create the ground body
ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)

# Create the particle emitter
emitter = sens.ChParticleEmitter()
emitter.SetEmissionRate(1)
emitter.SetParticlesPerEmission(1)
emitter.SetParticleSpeedMean(1)
emitter.SetParticleSpeedSigma(0.2)
emitter.SetParticleSizeMean(0.1)
emitter.SetParticleSizeSigma(0.01)

# Create the sensor manager
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(100,100,100),chrono.ChColor(intensity,intensity,intensity),1000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-100,-100,100),chrono.ChColor(intensity,intensity,intensity),1000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-100,100,-100),chrono.ChColor(intensity,intensity,intensity),1000.0)
manager.scene.AddPointLight(chrono.ChVector3f(100,-100,-100),chrono.ChColor(intensity,intensity,intensity),1000.0)

# Create a filter graph for the first sensor
filter_graph_1 = sens.ChFilterGraph()
filter_graph_1.AddFilter(sens.ChFilterVisualize(1280,720,"Filter Graph 1"))
filter_graph_1.SetName("Filter Graph 1")

# Create a camera and add it to the sensor manager
offset_pose = chrono.ChFramed(chrono.ChVector3d(-5,0,2))
cam = sens.ChCameraSensor(
    ground,                  # body camera is attached to
    update_rate,             # update rate in Hz
    offset_pose,             # offset pose
    image_width,             # image width
    image_height,            # image height
    fov                    # camera's horizontal field of view
)
cam.SetName("Camera Sensor 1")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)
manager.AddSensor(cam)
manager.SetFilterGraph(cam,filter_graph_1)

# Create a filter graph for the second sensor
filter_graph_2 = sens.ChFilterGraph()
filter_graph_2.AddFilter(sens.ChFilterVisualize(1280,720,"Filter Graph 2"))
filter_graph_2.SetName("Filter Graph 2")

# Create a camera and add it to the sensor manager
offset_pose = chrono.ChFramed(chrono.ChVector3d(-5,0,2))
cam = sens.ChCameraSensor(
    ground,                  # body camera is attached to
    update_rate,             # update rate in Hz
    offset_pose,             # offset pose
    image_width,             # image width
    image_height,            # image height
    fov                    # camera's horizontal field of view
)
cam.SetName("Camera Sensor 2")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)
manager.AddSensor(cam)
manager.SetFilterGraph(cam,filter_graph_2)

# Create a filter graph for the depth sensor
filter_graph_3 = sens.ChFilterGraph()
filter_graph_3.AddFilter(sens.ChFilterVisualize(1280,720,"Filter Graph 3"))
filter_graph_3.SetName("Filter Graph 3")

# Create a depth camera and add it to the sensor manager
offset_pose = chrono.ChFramed(chrono.ChVector3d(-5,0,2))
depth_cam = sens.ChDepthCamera(
    ground,                  # body camera is attached to
    update_rate,             # update rate in Hz
    offset_pose,             # offset pose
    image_width,             # image width
    image_height,            # image height
    fov                    # camera's horizontal field of view
)
depth_cam.SetName("Depth Camera Sensor")
depth_cam.SetLag(lag)
depth_cam.SetCollectionWindow(exposure_time)
depth_cam.SetDepthRange(0.1,10.0)
depth_cam.SetDepthQuantization(1e-3)
manager.AddSensor(depth_cam)
manager.SetFilterGraph(depth_cam,filter_graph_3)

# Create a filter graph for the lidar sensor
filter_graph_4 = sens.ChFilterGraph()
filter_graph_4.AddFilter(sens.ChFilterVisualizePointCloud(1280,720,"Filter Graph 4"))
filter_graph_4.SetName("Filter Graph 4")

# Create a lidar and add it to the sensor manager
offset_pose = chrono.ChFramed(chrono.ChVector3d(-5,0,2))
lidar = sens.ChLidarSensor(
    ground,                  # body lidar is attached to
    update_rate,             # update rate in Hz
    offset_pose,             # offset pose
    horizontal_samples,      # number of horizontal samples
    vertical_samples,        # number of vertical channels
    horizontal_fov,          # horizontal field of view
    max_vert_angle,
    min_vert_angle,
    100 # max lidar range
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(exposure_time)
lidar.SetReturnMode(sens.ChLidarReturnMode_STRONGEST_RETURN)
manager.AddSensor(lidar)
manager.SetFilterGraph(lidar,filter_graph_4)

# Create the simulation loop
orbit_radius = 5
orbit_rate = 0.2
ch_time = 0.0

while (ch_time < end_time):

    # Update the camera's position
    time = ch_time / orbit_rate
    cam.SetOffsetPose(chrono.ChFramed(
        chrono.ChVector3d(-orbit_radius * math.cos(time), -orbit_radius * math.sin(time), 1),
        chrono.QuatFromAngleAxis(time, chrono.ChVector3d(0,0,1)))
    )

    # Access the current sensor data
    accessacam1 = manager.GetMostRecentSample(cam)
    if (accessacam1.HasData()):
        rgba8_buffer = accessacam1.GetRGBA8Buffer()
        print('Camera 1 Resolution: ', accessacam1.ImageWidth, accessacam1.ImageHeight)
        print('Camera 1 Data Size: ', len(rgba8_buffer))

    accessacam2 = manager.GetMostRecentSample(cam)
    if (accessacam2.HasData()):
        rgba8_buffer = accessacam2.GetRGBA8Buffer()
        print('Camera 2 Resolution: ', accessacam2.ImageWidth, accessacam2.ImageHeight)
        print('Camera 2 Data Size: ', len(rgba8_buffer))

    accesslidar = manager.GetMostRecentSample(lidar)
    if (accesslidar.HasData()):
        float32_buffer = accesslidar.GetDepthBuffer()
        print('Lidar Resolution: ', accesslidar.ImageWidth, accesslidar.ImageHeight)
        print('Lidar Data Size: ', len(float32_buffer))

    # Update sensor manager
    # Will render/save/filter automatically
    manager.Update()

    # Perform step of dynamics
    step_size = 1e-3
    system.DoStepDynamics(step_size)

    # Get the current time of the simulation
    ch_time = system.GetChTime()

# This simulation setup demonstrates the use of multiple sensors in PyChrono.