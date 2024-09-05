import pychrono as ch
import pychrono.sensor as sens
import pychrono.ros as chros
import pychrono.mesh as mesh
import numpy as np


system = ch.ChSystemNSC()
system.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
ch.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
ch.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


vis_mesh = mesh.ChTriangleMeshConnected()
vis_mesh.LoadWavefrontMesh(ch.GetChronoDataFile("vehicle/hmmwv/hmmwv_body.obj"), False, True)
vis_mesh.Transform(ch.ChVector3d(0, 0, 1), ch.ChMatrix33d(1))

vis_body = ch.ChBodyMesh()
vis_body.SetName("HMMWV")
vis_body.SetPos(ch.ChVector3d(0, 0, 0))
vis_body.SetRot(ch.ChQuaterniond(1, 0, 0, 0))
vis_body.SetStatic(True)
vis_body.GetVisualShape(0).SetMesh(vis_mesh)
vis_body.GetVisualShape(0).SetName("HMMWV_mesh")
vis_body.GetVisualShape(0).SetMutable(False)
system.Add(vis_body)


ground = ch.ChBody()
ground.SetName("ground")
ground.SetPos(ch.ChVector3d(0, 0, 0))
ground.SetRot(ch.ChQuaterniond(1, 0, 0, 0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetMesh(vis_mesh)
ground.GetVisualShape(0).SetName("ground_mesh")
ground.GetVisualShape(0).SetMutable(False)
system.Add(ground)


manager = sens.ChSensorManager(system)


ros_manager = chros.ChROSPublisherManager()
ros_manager.RegisterPublisher(chros.ChROSPublisher("sensor_pub", 10))


offset_pose = ch.ChFramed(ch.ChVector3d(0.0, 0.0, 2.0), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 1, 0)))
image_width = 1280
image_height = 720
image_fps = 30
cam = sens.ChCameraSensor(ground, offset_pose, image_width, image_height, image_fps)
cam.SetName("Camera Sensor")
cam.SetCollectionWindow(1.0 / image_fps)
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Camera"))
cam.PushFilter(sens.ChFilterROSImage(ros_manager, "camera/image_raw", image_width, image_height, "bgr8"))
manager.AddSensor(cam)


offset_pose = ch.ChFramed(ch.ChVector3d(0.0, 0.0, 2.0), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(ground, offset_pose, 360, 1, 30)
lidar.SetName("Lidar Sensor")
lidar.SetCollectionWindow(1.0 / 30)
lidar.PushFilter(sens.ChFilterVisualize(640, 480, "Lidar"))
lidar.PushFilter(sens.ChFilterROSPointCloud(ros_manager, "lidar/pointcloud", 640, 480))
manager.AddSensor(lidar)


offset_pose = ch.ChFramed(ch.ChVector3d(0.0, 0.0, 2.0), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 1, 0)))
gps = sens.ChGPSSensor(ground, offset_pose)
gps.SetName("GPS Sensor")
gps.SetCollectionWindow(1.0)
gps.PushFilter(sens.ChFilterGPS())
gps.PushFilter(sens.ChFilterVisualizeGPS())
gps.PushFilter(sens.ChFilterROSPointCloud(ros_manager, "gps/fix"))
manager.AddSensor(gps)


offset_pose = ch.ChFramed(ch.ChVector3d(0.0, 0.0, 2.0), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 1, 0)))
accel = sens.ChAccelerometerSensor(ground, offset_pose)
accel.SetName("Accelerometer Sensor")
accel.SetCollectionWindow(1.0 / 30)
accel.PushFilter(sens.ChFilterVisualize(640, 480, "Accelerometer"))
accel.PushFilter(sens.ChFilterROSPointCloud(ros_manager, "imu/data"))
manager.AddSensor(accel)


offset_pose = ch.ChFramed(ch.ChVector3d(0.0, 0.0, 2.0), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 1, 0)))
gyro = sens.ChGyroscopeSensor(ground, offset_pose)
gyro.SetName("Gyroscope Sensor")
gyro.SetCollectionWindow(1.0 / 30)
gyro.PushFilter(sens.ChFilterVisualize(640, 480, "Gyroscope"))
gyro.PushFilter(sens.ChFilterROSPointCloud(ros_manager, "imu/data"))
manager.AddSensor(gyro)


offset_pose = ch.ChFramed(ch.ChVector3d(0.0, 0.0, 2.0), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 1, 0)))
mag = sens.ChMagnetometerSensor(ground, offset_pose)
mag.SetName("Magnetometer Sensor")
mag.SetCollectionWindow(1.0 / 30)
mag.PushFilter(sens.ChFilterVisualize(640, 480, "Magnetometer"))
mag.PushFilter(sens.ChFilterROSPointCloud(ros_manager, "imu/data"))
manager.AddSensor(mag)


manager.InitializeSensors()


time = 0
time_step = 1e-3
time_end = 30
while time < time_end:
    time = system.GetChTime()
    manager.Update()
    system.DoStepDynamics(time_step)
    ch.ChSleep(time_step)