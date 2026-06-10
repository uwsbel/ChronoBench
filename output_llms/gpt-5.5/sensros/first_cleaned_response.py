import math
import os
import time

import pychrono as chrono
import pychrono.sensor as sens
import pychrono.ros as chros






def make_frame(pos, rot=chrono.QUNIT):
    
    try:
        return chrono.ChFramed(pos, rot)
    except AttributeError:
        return chrono.ChFrameD(pos, rot)


def set_body_kinematics(body, pos, vel, acc, yaw, yaw_dt, yaw_ddt):
    
    body.SetPos(pos)
    body.SetRot(chrono.QuatFromAngleZ(yaw))

    
    if hasattr(body, "SetPosDt"):
        body.SetPosDt(vel)
    if hasattr(body, "SetPosDt2"):
        body.SetPosDt2(acc)

    
    w = chrono.ChVector3d(0, 0, yaw_dt)
    a = chrono.ChVector3d(0, 0, yaw_ddt)

    if hasattr(body, "SetAngVelParent"):
        body.SetAngVelParent(w)
    elif hasattr(body, "SetWvel_par"):
        body.SetWvel_par(w)

    if hasattr(body, "SetAngAccParent"):
        body.SetAngAccParent(a)
    elif hasattr(body, "SetWacc_par"):
        body.SetWacc_par(a)


def safe_push_filter(sensor, filt):
    
    try:
        sensor.PushFilter(filt)
    except Exception:
        pass


def register_ros_handler(ros_manager, handler):
    try:
        ros_manager.RegisterHandler(handler)
    except Exception as e:
        print(f"Could not register ROS handler {handler}: {e}")






system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

step_size = 1.0 / 200.0
end_time = 60.0


if "CHRONO_DATA_DIR" in os.environ:
    chrono.SetChronoDataPath(os.environ["CHRONO_DATA_DIR"])







floor = chrono.ChBody()
floor.SetName("world_floor")
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, 0, -0.05))

floor_shape = chrono.ChVisualShapeBox(80, 80, 0.1)
floor_shape.SetName("floor_visual")
floor.AddVisualShape(floor_shape)

system.Add(floor)


ground = chrono.ChBody()
ground.SetName("moving_ground_sensor_platform")
ground.SetFixed(True)        
ground.EnableCollision(False)
ground.SetPos(chrono.ChVector3d(0, 0, 0))

platform_shape = chrono.ChVisualShapeBox(2.5, 1.5, 0.2)
platform_shape.SetName("platform_visual")
ground.AddVisualShape(platform_shape, make_frame(chrono.ChVector3d(0, 0, 0)))

system.Add(ground)


mesh_body = chrono.ChBody()
mesh_body.SetName("visual_mesh_target")
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(8, 0, 1.0))

try:
    tri_mesh = chrono.ChTriangleMeshConnected()
    
    tri_mesh.LoadWavefrontMesh(
        chrono.GetChronoDataFile("models/cube.obj"),
        False,
        True,
    )

    mesh_shape = chrono.ChVisualShapeTriangleMesh()
    mesh_shape.SetMesh(tri_mesh)
    mesh_shape.SetName("loaded_obj_mesh")
    mesh_body.AddVisualShape(mesh_shape)

except Exception:
    
    fallback_shape = chrono.ChVisualShapeBox(2.0, 2.0, 2.0)
    fallback_shape.SetName("fallback_box_mesh_target")
    mesh_body.AddVisualShape(fallback_shape)

system.Add(mesh_body)


for i, y in enumerate([-4, -2, 2, 4]):
    box = chrono.ChBody()
    box.SetName(f"static_box_{i}")
    box.SetFixed(True)
    box.SetPos(chrono.ChVector3d(12 + i * 3, y, 1.0))
    box.AddVisualShape(chrono.ChVisualShapeBox(1.0, 1.0, 2.0))
    system.Add(box)






manager = sens.ChSensorManager(system)


try:
    manager.scene.AddPointLight(
        chrono.ChVector3f(10, -10, 10),
        chrono.ChColor(1.0, 1.0, 1.0),
        500.0,
    )
    manager.scene.AddPointLight(
        chrono.ChVector3f(-10, 10, 8),
        chrono.ChColor(0.8, 0.8, 0.8),
        300.0,
    )
except Exception:
    pass


camera_pose = make_frame(
    chrono.ChVector3d(0.6, 0.0, 1.2),
    chrono.QUNIT,
)

lidar_pose = make_frame(
    chrono.ChVector3d(0.4, 0.0, 1.3),
    chrono.QUNIT,
)

imu_pose = make_frame(
    chrono.ChVector3d(0.0, 0.0, 0.4),
    chrono.QUNIT,
)


camera = sens.ChCameraSensor(
    ground,
    30.0,               
    camera_pose,
    1280,               
    720,                
    math.radians(70.0)  
)
camera.SetName("front_camera")
camera.SetLag(0.0)
camera.SetCollectionWindow(0.0)
safe_push_filter(camera, sens.ChFilterRGBA8Access())
manager.AddSensor(camera)


lidar = sens.ChLidarSensor(
    ground,
    10.0,                
    lidar_pose,
    2048,                
    32,                  
    math.radians(90.0),  
    math.radians(10.0),  
    math.radians(-10.0), 
    100.0                
)
lidar.SetName("front_lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)
safe_push_filter(lidar, sens.ChFilterPCfromDepth())
safe_push_filter(lidar, sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


noise_none = sens.ChNoiseNone()


gps_reference = chrono.ChVector3d(43.0731, -89.4012, 260.0)

gps = sens.ChGPSSensor(
    ground,
    10.0,
    imu_pose,
    gps_reference,
    noise_none,
)
gps.SetName("gps")
gps.SetLag(0.0)
gps.SetCollectionWindow(0.0)
manager.AddSensor(gps)

accelerometer = sens.ChAccelerometerSensor(
    ground,
    100.0,
    imu_pose,
    noise_none,
)
accelerometer.SetName("accelerometer")
accelerometer.SetLag(0.0)
accelerometer.SetCollectionWindow(0.0)
manager.AddSensor(accelerometer)

gyroscope = sens.ChGyroscopeSensor(
    ground,
    100.0,
    imu_pose,
    noise_none,
)
gyroscope.SetName("gyroscope")
gyroscope.SetLag(0.0)
gyroscope.SetCollectionWindow(0.0)
manager.AddSensor(gyroscope)


try:
    magnetometer = sens.ChMagnetometerSensor(
        ground,
        50.0,
        imu_pose,
        noise_none,
        gps_reference,
    )
except TypeError:
    magnetometer = sens.ChMagnetometerSensor(
        ground,
        50.0,
        imu_pose,
        gps_reference,
        noise_none,
    )

magnetometer.SetName("magnetometer")
magnetometer.SetLag(0.0)
magnetometer.SetCollectionWindow(0.0)
manager.AddSensor(magnetometer)






ros_manager = chros.ChROSManager()


register_ros_handler(ros_manager, chros.ChROSClockHandler())


register_ros_handler(
    ros_manager,
    chros.ChROSCameraHandler(camera, "/chrono/camera/image"),
)

register_ros_handler(
    ros_manager,
    chros.ChROSLidarHandler(lidar, "/chrono/lidar/points"),
)

register_ros_handler(
    ros_manager,
    chros.ChROSGPSHandler(gps, "/chrono/gps/fix"),
)

register_ros_handler(
    ros_manager,
    chros.ChROSAccelerometerHandler(accelerometer, "/chrono/imu/accel"),
)

register_ros_handler(
    ros_manager,
    chros.ChROSGyroscopeHandler(gyroscope, "/chrono/imu/gyro"),
)

register_ros_handler(
    ros_manager,
    chros.ChROSMagnetometerHandler(magnetometer, "/chrono/imu/mag"),
)


try:
    register_ros_handler(
        ros_manager,
        chros.ChROSBodyHandler(50.0, ground, "/chrono/platform/state"),
    )
except Exception:
    pass

if not ros_manager.Initialize():
    raise RuntimeError("Failed to initialize Chrono ROS manager.")






realtime_timer = chrono.ChRealtimeStepTimer()

print("Starting PyChrono sensor + ROS simulation.")
print("Publishing:")
print("  /chrono/camera/image")
print("  /chrono/lidar/points")
print("  /chrono/gps/fix")
print("  /chrono/imu/accel")
print("  /chrono/imu/gyro")
print("  /chrono/imu/mag")

while system.GetChTime() < end_time:
    t = system.GetChTime()

    
    
    vx = 1.0
    x = vx * t
    x_dt = vx
    x_ddt = 0.0

    Ay = 1.5
    wy = 0.35
    y = Ay * math.sin(wy * t)
    y_dt = Ay * wy * math.cos(wy * t)
    y_ddt = -Ay * wy * wy * math.sin(wy * t)

    z = 0.0
    z_dt = 0.0
    z_ddt = 0.0

    Ayaw = math.radians(10.0)
    wyaw = 0.25
    yaw = Ayaw * math.sin(wyaw * t)
    yaw_dt = Ayaw * wyaw * math.cos(wyaw * t)
    yaw_ddt = -Ayaw * wyaw * wyaw * math.sin(wyaw * t)

    pos = chrono.ChVector3d(x, y, z)
    vel = chrono.ChVector3d(x_dt, y_dt, z_dt)
    acc = chrono.ChVector3d(x_ddt, y_ddt, z_ddt)

    set_body_kinematics(
        ground,
        pos,
        vel,
        acc,
        yaw,
        yaw_dt,
        yaw_ddt,
    )

    
    manager.Update()

    
    if not ros_manager.Update(t, step_size):
        break

    
    system.DoStepDynamics(step_size)

    
    realtime_timer.Spin(step_size)

print("Simulation finished.")