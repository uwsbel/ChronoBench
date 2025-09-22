import pychrono as chrono
import pychrono.sensor as sens
import pychrono.vehicle  # For easier visualization mesh loading
import pychrono.irrlicht as irr

import time

# ROS imports
try:
    import pychrono.sensor_ros as chrono_ros
except ImportError:
    chrono_ros = None
    print("[Warning] pychrono.sensor_ros not available. ROS integration disabled.")

def main():

    # 1. Initialize the chrono system
    chrono.SetChronoDataPath(chrono.GetChronoDataPath() or "./")  # Make sure data path is set

    # Create physical system
    system = chrono.ChSystemNSC()

    # Set gravity
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # 2. Create ground body
    ground = chrono.ChBody()
    ground.SetBodyFixed(False)  # Allow the ground to move as per instructions
    ground.SetMass(1000)  # Give some mass to the ground body, so that forces matter
    ground.SetInertiaXX(chrono.ChVectorD(100, 100, 100))
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    ground.SetPos_dt(chrono.ChVectorD(0.5, 0.0, 0))  # initial velocity to move ground a bit
    ground.SetBodyFixed(False)
    system.Add(ground)

    # Add visualization mesh to ground - e.g., a simple box mesh or an external file
    mesh_file = chrono.GetChronoDataFile('sensor/textures/box.obj')  # Using a box mesh obj for example
    # If the mesh file does not exist, use a box shape visualization as a fallback
    try:
        ground_mesh = chrono.ChTriangleMeshShape()
        ground_mesh.SetMesh(chrono.ChTriangleMeshShape().LoadWavefrontMesh(mesh_file, True, False))
        ground_mesh.SetName("GroundMesh")
        ground_mesh.SetStatic(True)
        ground.GetAssets().push_back(ground_mesh)
    except Exception:
        # Fallback to box shape
        box_shape = chrono.ChBoxShape()
        box_shape.GetBoxGeometry().Size = chrono.ChVectorD(5, 5, 0.1)
        ground.AddAsset(box_shape)

    # 3. Setup the Irrlicht visualization (optional but helpful to visualize the scene)
    application = irr.ChIrrApp(system, "PyChrono Multisensor Example", irr.dimension2du(1280, 720))
    application.AddTypicalSky()
    application.AddTypicalLogo()
    application.AddTypicalLights()
    application.AddTypicalCamera(chrono.ChVectorD(-10, -10, 10), chrono.ChVectorD(0, 0, 0))

    application.AssetBindAll()
    application.AssetUpdateAll()

    # 4. SENSOR SETUP

    # Create a sensor manager to handle sensors on the ground body
    sensor_manager = sens.ChSensorManager(system)

    # Attach the sensor manager system to the Irrlicht app (for rendering sensor outputs if needed)
    sensor_manager.SetChVisualSystem(application.GetVisualSystem())

    # Common sensor pose relative to ground
    sensor_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.8))  # ~1.8m height on ground

    # Sensor parameters
    time_step = 1.0 / 30       # 30 Hz sensor rate

    # --- CAMERA SENSOR ---
    cam_width = 640
    cam_height = 480
    fov = 70

    camera = sens.ChCameraSensor(
        ground,
        time_step,
        sens.ChFrameD(chrono.ChVectorD(0, 0, 1.8)),  # position relative to ground
        cam_width,
        cam_height,
        fov)
    camera.PushFilter(sens.ChFilterRGBA8Access())
    sensor_manager.AddSensor(camera)

    # --- LIDAR SENSOR ---
    lidar = sens.ChLidarSensor(
        ground,
        time_step,
        sens.ChFrameD(chrono.ChVectorD(0, 0, 1.8)),
        100,     # samples per scan sweep (azimuth resolution)
        0.01,    # scan duration (s)
        360,     # horizontal FOV deg, full 360 deg sweep
        16,      # vertical channels
        -15,     # vertical FOV min deg
        15,      # vertical FOV max deg
        100.0)   # max distance in meters
    sensor_manager.AddSensor(lidar)

    # --- GPS SENSOR ---
    gps = sens.ChGPSSensor(
        ground,
        time_step,
        sens.ChFrameD(sensor_pose.GetPos()),
        1.0,    # noise sigma in meters
        1.0)    # bias in meters, zero bias for now
    sensor_manager.AddSensor(gps)

    # --- ACCELEROMETER SENSOR ---
    accelerometer = sens.ChAccelerometerSensor(
        ground,
        time_step,
        sensor_pose,
        0.01)   # noise sigma (m/s²)
    sensor_manager.AddSensor(accelerometer)

    # --- GYROSCOPE SENSOR ---
    gyro = sens.ChGyroscopeSensor(
        ground,
        time_step,
        sensor_pose,
        0.001)  # noise sigma (rad/s)
    sensor_manager.AddSensor(gyro)

    # --- MAGNETOMETER SENSOR ---
    magnetometer = sens.ChMagnetometerSensor(
        ground,
        time_step,
        sensor_pose,
        0.01)  # noise sigma in microTesla (uT)
    sensor_manager.AddSensor(magnetometer)

    # 5. Setup ROS manager for publishing sensor outputs to ROS topics
    ros_manager = None
    if chrono_ros:
        ros_manager = chrono_ros.ChROS2Interface.Create()
        ros_manager.Initialize()

        # Register each sensor with appropriate ROS topic
        ros_manager.RegisterSensor(camera, "/camera/image_raw")
        ros_manager.RegisterSensor(lidar, "/lidar/points")
        ros_manager.RegisterSensor(gps, "/gps/fix")
        ros_manager.RegisterSensor(accelerometer, "/imu/accel")
        ros_manager.RegisterSensor(gyro, "/imu/gyro")
        ros_manager.RegisterSensor(magnetometer, "/imu/mag")
    else:
        print("[Warning] ROS manager not initialized because pychrono.sensor_ros is unavailable.")

    # 6. Initialize sensor manager before simulation loop
    sensor_manager.Initialize()

    # 7. Simulation loop parameters
    simulation_step = 0.001  # physics step (1ms)
    simulation_time = 10.0   # run simulation for 10 seconds

    t = 0
    last_time = time.time()

    # Use application for visualization (optional)
    application.SetTimestep(simulation_step)

    print("Starting simulation...")

    while t < simulation_time:
        # Read real time before step
        start_sim_time = time.time()

        # Update sensors
        sensor_manager.Update()

        # Advance simulation by physics step
        system.DoStepDynamics(simulation_step)

        # Update ROS manager: publish messages
        if ros_manager:
            ros_manager.Update()

        # Irrlicht visualization step
        if application.GetDevice().run():
            application.BeginScene()
            application.DrawAll()
            application.EndScene()
        else:
            # Window closed, break simulation
            break

        # Maintain real-time (sleep if simulation step is too fast)
        elapsed = time.time() - start_sim_time
        sleep_time = simulation_step - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

        # Update simulation time counter
        t += simulation_step

    print("Simulation ended.")


if __name__ == "__main__":
    main()