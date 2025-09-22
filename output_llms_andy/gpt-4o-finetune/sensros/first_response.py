import os
import math
import numpy as np
import pychrono as chrono
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'pyros'))
from pyros import r_manager, r_sensor

def main():
    SCENE_FOLDER = sens.GetChronoDataFile("sensor/scene/").sys.data_dir

    # Create the Chrono simulation system
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.Initialize()

    # Add a visual mesh to the scene for enhanced visualization
    trimesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
        sens.GetChronoDataFile("mesh/vehicle/quadcopter.obj"), False, True)
    trimesg_shape = chrono.ChVisualShapeTriangleMesh()
    trimesg_shape.SetMesh(trimesh)
    trimesg_shape.SetName("Quadcopter Mesh")
    trimesg_shape.SetMutable(False)

    # Create a body to hold the visual mesh
    mbody = chrono.ChBody()
    mbody.SetPos(chrono.ChVector3d(0, 0, 0))
    mbody.AddVisualShape(trimesg_shape)
    mbody.SetFixed(False)
    sys.Add(mbody)

    # Create the ground body, which will be the reference for all sensors
    ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)
    ground_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/grid.png"))
    sys.Add(ground_body)

    # Attach various sensors to the ground body

    # 1. Camera sensor
    cam_offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 2), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(
        ground_body,                               # Body this camera is attached to
        30,                                        # Camera update rate in Hz
        cam_offset_pose,                           # Offset pose of the camera
        1280,                                      # Image width
        720,                                       # Image height
        1.408,                                     # Camera's horizontal field of view
        "rgba",                                    # Output image format
        sens.SENSOR_LOG_MODE_SEQUENCE                 # Log mode for the camera
    )
    cam.PushFilter(sens.ChFilterVisualize(1280, 720))  # Visualize image
    cam.PushFilter(r_sensor.ChFilterROSPublishImage().SetTopic("mod_cam/image_rgba"))  # Publish image to ROS topic
    manager.AddSensor(cam)

    # 2. Lidar sensor
    lidar_offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 2), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        ground_body,                               # Body lidar is attached to
        10,                                        # Scanning rate in Hz
        lidar_offset_pose,                         # Offset pose of the lidar
        100,                                       # Number of horizontal samples
        2 * chrono.CH_PI,                          # Horizontal field of view
        300,                                       # Number of vertical channels
        -chrono.CH_PI / 12,                        # Minimum vertical angle
        +chrono.CH_PI / 6,                         # Maximum vertical angle
        100.0,                                     # Maximum lidar range
        sens.LIDAR_DISTANCE,                       # Lidar return mode
        0.01                                       # Lag time of the sample
    )
    lidar.PushFilter(sens.ChFilterDIAccess())  # Access lidar data directly
    lidar.PushFilter(r_sensor.ChFilterROSPublishPCD().SetTopic("mod_lidar/pointcloud"))  # Publish point cloud to ROS topic
    manager.AddSensor(lidar)

    # 3. GPS sensor
    gps_reference = chrono.ChVector3d(-89.4, 433.07, 200.0)  # GPS reference point
    gps = sens.ChGPSSensor(
        ground_body,                               # Body GPS is attached to
        10,                                        # Update rate in Hz
        chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
        gps_reference,                             # GPS reference point
        sens.GPSACCURACY_HIGH                       # GPS accuracy
    )
    gps.PushFilter(r_sensor.ChFilterROSPublishA2B().SetTopic("mod_gps/data"))  # Publish GPS data to ROS topic
    manager.AddSensor(gps)

    # 4. Accelerometer sensor
    acc = sens.ChAccelerometerSensor(ground_body, 10, chrono.ChFramed())
    acc.PushFilter(r_sensor.ChFilterROSPublishA2B().SetTopic("mod_acc/data"))  # Publish accelerometer data to ROS topic
    manager.AddSensor(acc)

    # 5. Gyroscope sensor
    gyro = sens.ChGyroscopeSensor(ground_body, 10, chrono.ChFramed())
    gyro.PushFilter(r_sensor.ChFilterROSPublishA2B().SetTopic("mod_gyro/data"))  # Publish gyroscope data to ROS topic
    manager.AddSensor(gyro)

    # 6. Magnetometer sensor
    mag = sens.ChMagnetometerSensor(ground_body, 10, chrono.ChFramed())
    mag.PushFilter(r_sensor.ChFilterROSPublishA2B().SetTopic("mod_mag/data"))  # Publish magnetometer data to ROS topic
    manager.AddSensor(mag)

    # Create and configure the ROS manager
    ros_manager = r_manager.ChROSPythonManager()
    ros_manager.AddSensor(cam)
    ros_manager.AddSensor(lidar)
    ros_manager.AddSensor(gps)
    ros_manager.AddSensor(acc)
    ros_manager.AddSensor(gyro)
    ros_manager.AddSensor(mag)

    # Register all sensors with the ROS manager for publishing
    if not ros_manager.Initialize():
        print("Error initializing the ROS manager.")
        return False

    # Simulation loop
    time = 0
    time_step = 1e-3  # Simulation time step
    time_end = 100    # Duration of the simulation

    # Variables for ground body movement
    radius = 4
    omega = 2 * chrono.CH_PI / 20  # Angular velocity

    while time < time_end:
        time = sys.GetChTime()  # Update simulation time

        # Circular motion of the ground body
        ground_body.SetPos(chrono.ChVector3d(radius * math.cos(omega * time), radius * math.sin(omega * time), 0.0))

        manager.Update()  # Update all sensors

        # Update ROS manager and check for termination condition
        if not ros_manager.Update(time, time_step):
            break

        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step

if __name__ == "__main__":
    main()