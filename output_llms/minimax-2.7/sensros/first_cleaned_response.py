import pychrono as chrono
import pychrono.sensor as sensormanager
import pychrono.ros as chrono_ros
import numpy as np
import time


class SimulationController:
    
    
    def __init__(self, timestep=0.001, real_time_factor=1.0):
        self.time_step = timestep
        self.real_time_factor = real_time_factor
        self.simulation_time = 0.0
        self.frame_count = 0
        self.start_wall_time = None
        
    def sleep_until_realtime(self):
        
        if self.start_wall_time is None:
            self.start_wall_time = time.time()
            
        expected_time = self.simulation_time / self.real_time_factor
        elapsed_time = time.time() - self.start_wall_time
        sleep_time = expected_time - elapsed_time
        
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    def advance(self):
        
        self.simulation_time += self.time_step
        self.frame_count += 1


class GroundMover:
    
    
    def __init__(self, body):
        self.body = body
        self.motion_type = "sinusoidal"  
        
    def update(self, time):
        
        if self.motion_type == "sinusoidal":
            
            amplitude = 0.5  
            frequency = 0.5  
            x = amplitude * np.sin(2 * np.pi * frequency * time)
            z = amplitude * np.cos(2 * np.pi * frequency * time)
            self.body.SetPos(chrono.ChVectorD(x, 0.0, z))
            
            
            vx = amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * time)
            vz = -amplitude * 2 * np.pi * frequency * np.sin(2 * np.pi * frequency * time)
            self.body.SetPos_dt(chrono.ChVectorD(vx, 0.0, vz))
            
        elif self.motion_type == "linear":
            
            speed = 0.5  
            x = speed * time
            self.body.SetPos(chrono.ChVectorD(x, 0.0, 0.0))
            self.body.SetPos_dt(chrono.ChVectorD(speed, 0.0, 0.0))
            
        elif self.motion_type == "circular":
            
            radius = 0.5  
            omega = 1.0  
            x = radius * np.cos(omega * time)
            z = radius * np.sin(omega * time)
            self.body.SetPos(chrono.ChVectorD(x, 0.0, z))
            
            
            self.body.SetAngularVelocity(chrono.ChVectorD(0, omega, 0))


def create_ground_body(system):
    
    
    
    ground_mat = chrono.ChMaterialSurfaceNSC()
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0.1)
    
    
    ground_body = chrono.ChBody()
    ground_body.SetName("Ground_Body")
    ground_body.SetBodyFixed(False)  
    ground_body.SetMass(10.0)
    ground_body.SetInertiaXX(chrono.ChVectorD(1.0, 1.0, 1.0))
    ground_body.SetPos(chrono.ChVectorD(0.0, 0.5, 0.0))
    ground_body.SetMaterialSurface(ground_mat)
    
    
    ground_body.GetCollisionModel().SetBox(0.5, 0.1, 0.5)
    ground_body.GetCollisionModel().BuildModel()
    
    
    system.AddBody(ground_body)
    
    return ground_body


def create_ground_mesh():
    
    
    
    mesh = chrono.ChTriangleMeshConnected()
    
    
    hx, hy, hz = 0.5, 0.1, 0.5  
    vertices = [
        chrono.ChVectorD(-hx, -hy, -hz),
        chrono.ChVectorD( hx, -hy, -hz),
        chrono.ChVectorD( hx,  hy, -hz),
        chrono.ChVectorD(-hx,  hy, -hz),
        chrono.ChVectorD(-hx, -hy,  hz),
        chrono.ChVectorD( hx, -hy,  hz),
        chrono.ChVectorD( hx,  hy,  hz),
        chrono.ChVectorD(-hx,  hy,  hz),
    ]
    
    
    faces = [
        [0, 1, 2], [0, 2, 3],  
        [4, 6, 5], [4, 7, 6],  
        [0, 4, 5], [0, 5, 1],  
        [2, 6, 7], [2, 7, 3],  
        [0, 3, 7], [0, 7, 4],  
        [1, 5, 6], [1, 6, 2],  
    ]
    
    
    for v in vertices:
        mesh.AddNode(chrono.ChVectorD(v.x, v.y, v.z))
    
    for f in faces:
        triangle = chrono.ChTriangle()
        triangle.SetNode(0, chrono.ChVectorD(vertices[f[0]].x, vertices[f[0]].y, vertices[f[0]].z))
        triangle.SetNode(1, chrono.ChVectorD(vertices[f[1]].x, vertices[f[1]].y, vertices[f[1]].z))
        triangle.SetNode(2, chrono.ChVectorD(vertices[f[2]].x, vertices[f[2]].y, vertices[f[2]].z))
        mesh.AddTriangle(triangle)
    
    
    visual_mat = chrono.ChVisualizationTriangleMeshShape()
    visual_mat.SetMesh(mesh)
    visual_mat.SetColor(chrono.ChColor(0.5, 0.5, 0.5))  
    
    return mesh, visual_mat


def setup_camera_sensor(manager, parent_body, offset):
    
    
    
    update_rate = 30  
    horizontal_fov = 1.2  
    image_width = 1280
    image_height = 720
    min_distance = 0.1
    max_distance = 100.0
    
    
    camera = sensormanager.ChCameraSensor(
        parent_body,
        update_rate,
        chrono.ChFrameD(offset),
        horizontal_fov,
        image_width,
        image_height,
        min_distance,
        max_distance
    )
    
    
    camera.SetName("Camera_Sensor")
    camera.SetLocalOffset(chrono.ChVectorD(0.0, 0.3, 0.0))
    
    
    camera.SetOverlayColor(chrono.ChColor(1, 1, 1))
    
    
    manager.Add(camera)
    
    return camera


def setup_lidar_sensor(manager, parent_body, offset):
    
    
    
    update_rate = 20  
    num_layers = 16
    points_per_layer = 1800
    max_distance = 50.0
    vertical_fov = 0.3  
    horizontal_fov = 2 * np.pi  
    
    
    lidar = sensormanager.ChLidarSensor(
        parent_body,
        update_rate,
        chrono.ChFrameD(offset),
        num_layers,
        points_per_layer,
        max_distance,
        vertical_fov,
        horizontal_fov
    )
    
    
    lidar.SetName("Lidar_Sensor")
    lidar.SetLocalOffset(chrono.ChVectorD(0.0, 0.25, 0.0))
    
    
    scan_freq = 10  
    lidar.SetScanFrequency(scan_freq)
    
    
    lidar.SetNoiseLite(True)
    
    
    manager.Add(lidar)
    
    return lidar


def setup_gps_sensor(manager, parent_body, offset):
    
    
    
    update_rate = 10  
    
    
    gps = sensormanager.ChGPSSensor(
        parent_body,
        update_rate,
        chrono.ChFrameD(offset)
    )
    
    
    gps.SetName("GPS_Sensor")
    gps.SetLocalOffset(chrono.ChVectorD(0.0, 0.2, 0.0))
    
    
    gps.SetNoiseGPS(True)
    gps.SetNoiseGyro(True)
    
    
    manager.Add(gps)
    
    return gps


def setup_imu_sensor(manager, parent_body, offset):
    
    
    
    update_rate = 100  
    
    
    imu = sensormanager.ChIMUSensor(
        parent_body,
        update_rate,
        chrono.ChFrameD(offset)
    )
    
    
    imu.SetName("IMU_Sensor")
    imu.SetLocalOffset(chrono.ChVectorD(0.0, 0.1, 0.0))
    
    
    imu.SetNoiseAccel(True)
    imu.SetNoiseGyro(True)
    imu.SetNoiseMag(True)
    
    
    imu.SetProcessNoiseAccel(0.05)   
    imu.SetProcessNoiseGyro(0.0005)   
    imu.SetProcessNoiseMag(0.01)      
    
    
    manager.Add(imu)
    
    return imu


def setup_sensor_manager(system):
    
    
    
    manager = sensormanager.ChSensorManager(system)
    
    
    manager.SetRenderingMode(sensormanager.RenderingMode_FULL)
    
    
    manager.SetAmbientLight(0.5)
    
    
    scene = manager.GetScene()
    scene.SetBackgroundColor(chrono.ChColor(0.1, 0.1, 0.2))  
    
    
    scene.AddPointLight(
        chrono.ChVectorD(5, 10, 5),
        chrono.ChColor(1, 1, 1),
        100.0
    )
    
    return manager


def setup_environment(system):
    
    
    
    ground_viz = chrono.ChVisualizationShape()
    ground_viz.AddBox(1.0, 0.02, 1.0)
    ground_viz.SetColor(chrono.ChColor(0.3, 0.8, 0.3))
    
    return ground_viz


def create_ros_interface(sensors, node_name="pychrono_sensor_node"):
    
    
    
    ros_manager = chrono_ros.ChROSMgr()
    
    
    ros_manager.Initialize(node_name, 1)  
    
    
    sensor_topic_map = {
        "Camera_Sensor": "/sensor/camera/image",
        "Lidar_Sensor": "/sensor/lidar/pointcloud",
        "GPS_Sensor": "/sensor/gps/fix",
        "IMU_Sensor": "/sensor/imu/data",
    }
    
    
    for sensor_name, topic in sensor_topic_map.items():
        if sensor_name in sensors:
            sensor = sensors[sensor_name]
            
            
            if "Camera" in sensor_name:
                msg_type = chrono_ros.ChROSMsgType_IMAGE
            elif "Lidar" in sensor_name:
                msg_type = chrono_ros.ChROSMsgType_POINTCLOUD
            elif "GPS" in sensor_name:
                msg_type = chrono_ros.ChROSMsgType_NAV_SAT_FIX
            elif "IMU" in sensor_name:
                msg_type = chrono_ros.ChROSMsgType_IMU
            else:
                msg_type = chrono_ros.ChROSMsgType_UNKNOWN
            
            
            ros_manager.RegisterSensor(sensor, topic, msg_type)
            print(f"Registered {sensor_name} on topic {topic}")
    
    return ros_manager


def run_simulation():
    
    
    print("=" * 60)
    print("PyChrono Simulation with Sensors and ROS Integration")
    print("=" * 60)
    
    
    print("\n[1] Initializing PyChrono environment...")
    
    
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.002)
    
    
    print("[2] Creating Chrono physical system...")
    
    my_system = chrono.ChSystemNSC()
    my_system.SetTitle("PyChrono Sensor Simulation")
    my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    my_system.SetSolverMaxIterations(500)
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
    
    
    print("[3] Creating ground body...")
    
    ground_body = create_ground_body(my_system)
    print(f"    Ground body created at position: {ground_body.GetPos()}")
    
    
    print("[4] Configuring ground body motion...")
    
    ground_mover = GroundMover(ground_body)
    ground_mover.motion_type = "sinusoidal"
    print(f"    Motion type: {ground_mover.motion_type}")
    
    
    print("[5] Setting up sensor manager...")
    
    sensor_manager = setup_sensor_manager(my_system)
    
    
    print("[6] Adding sensors to ground body...")
    
    sensors = {}
    sensor_offset = chrono.ChVectorD(0.0, 0.0, 0.0)
    
    
    print("    Adding Camera sensor...")
    camera = setup_camera_sensor(sensor_manager, ground_body, sensor_offset)
    sensors["Camera_Sensor"] = camera
    
    
    print("    Adding Lidar sensor...")
    lidar = setup_lidar_sensor(sensor_manager, ground_body, sensor_offset)
    sensors["Lidar_Sensor"] = lidar
    
    
    print("    Adding GPS sensor...")
    gps = setup_gps_sensor(sensor_manager, ground_body, sensor_offset)
    sensors["GPS_Sensor"] = gps
    
    
    print("    Adding IMU sensor (accelerometer, gyroscope, magnetometer)...")
    imu = setup_imu_sensor(sensor_manager, ground_body, sensor_offset)
    sensors["IMU_Sensor"] = imu
    
    print(f"    Total sensors added: {len(sensors)}")
    
    
    print("[7] Setting up ROS interface...")
    
    try:
        ros_interface = create_ros_interface(sensors)
        ros_enabled = True
        print("    ROS interface initialized successfully")
    except Exception as e:
        ros_enabled = False
        print(f"    Warning: ROS interface not available ({str(e)})")
        print("    Continuing simulation without ROS...")
    
    
    print("[8] Initializing simulation controller...")
    
    controller = SimulationController(
        timestep=0.001,  
        real_time_factor=1.0  
    )
    
    
    simulation_duration = 10.0  
    visualization_interval = 1.0  
    
    print("\n" + "=" * 60)
    print("Starting Simulation")
    print("=" * 60)
    print(f"Duration: {simulation_duration} seconds")
    print(f"Timestep: {controller.time_step} seconds")
    print(f"Real-time factor: {controller.real_time_factor}")
    print("-" * 60)
    
    
    print("\n[9] Running simulation loop...")
    
    try:
        while controller.simulation_time < simulation_duration:
            
            ground_mover.update(controller.simulation_time)
            
            
            sensor_manager.Update()
            
            
            my_system.DoStepDynamics(controller.time_step)
            
            
            controller.advance()
            
            
            controller.sleep_until_realtime()
            
            
            if controller.simulation_time % visualization_interval < controller.time_step:
                print(f"Time: {controller.simulation_time:.2f}s | "
                      f"Ground Pos: ({ground_body.GetPos().x:.3f}, "
                      f"{ground_body.GetPos().y:.3f}, "
                      f"{ground_body.GetPos().z:.3f}) | "
                      f"Frames: {controller.frame_count}")
                
                
                if ros_enabled:
                    ros_interface.SpinOnce()
    
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    
    
    print("\n" + "=" * 60)
    print("Simulation Complete")
    print("=" * 60)
    print(f"Total simulation time: {controller.simulation_time:.2f} seconds")
    print(f"Total frames: {controller.frame_count}")
    print(f"Average FPS: {controller.frame_count / controller.simulation_time:.2f}")
    
    
    if ros_enabled:
        print("Shutting down ROS interface...")
        ros_interface.Shutdown()
    
    print("\nSimulation finished successfully!")


if __name__ == "__main__":
    run_simulation()