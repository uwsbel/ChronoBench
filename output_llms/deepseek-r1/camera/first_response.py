import pychrono as chrono
import pychrono.irrlicht as irrlicht
import pychrono.sensor as sensor
import numpy as np
import os

# Set data path for assets
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))

# Initialize simulation system
system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, -9.81, 0))

# Create fixed body for mesh
mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
system.Add(mesh_body)

# Load mesh from OBJ file and attach to body
mesh_path = "models/triangular_mesh.obj"  # Replace with actual path
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_path)
mesh_shape = chrono.ChTriangleMeshShape(mesh)
mesh_shape.SetName("MeshVisualization")
mesh_body.AddAsset(mesh_shape)

# Create Irrlicht visualization
application = irrlicht.ChIrrApp(system, "Camera Sensor Demo", 800, 600)
application.AddTypicalSky()
application.AddTypicalCamera(chrono.ChVectorD(2, 2, -3))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()

# Initialize sensor manager
manager = sensor.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(10, 10, 10), chrono.ChColor(1, 1, 1), 1000)

# Camera orbit parameters
orbit_radius = 3.0
orbit_height = 1.0
angular_speed = 0.1  # rad/s

# Create camera sensor
camera = sensor.ChCameraSensor(
    mesh_body,              # Parent body
    30,                     # Update rate (Hz)
    chrono.ChFrameD(chrono.ChVectorD(orbit_radius, orbit_height, 0), 
                   chrono.Q_from_AngZ(np.pi/2)),  # Initial pose
    1280,                   # Image width
    720,                    # Image height
    np.deg2rad(75)          # Field of view
)
camera.SetName("Orbiting Camera")

# Configure camera settings
camera.PushFilter(sensor.ChFilterRGBA8Access())
camera.PushFilter(sensor.ChFilterVisualize(1280, 720, "Camera View"))
camera.PushFilter(sensor.ChFilterNoiseNormal(0.0, 0.02))  # Gaussian noise

manager.AddSensor(camera)

# Simulation parameters
time_step = 0.01
current_time = 0.0
orbit_angle = 0.0

# Main simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    # Update camera orbit position
    orbit_angle += angular_speed * time_step
    cam_x = orbit_radius * np.cos(orbit_angle)
    cam_z = orbit_radius * np.sin(orbit_angle)
    cam_pos = chrono.ChVectorD(cam_x, orbit_height, cam_z)
    
    # Point camera towards mesh origin
    cam_rot = chrono.ChMatrix33D()
    cam_rot.Set_A_Raxis(-cam_pos, chrono.ChVectorD(0, 1, 0))
    camera.SetOffsetPose(chrono.ChFrameD(cam_pos, cam_rot.Get_A_quaternion()))
    
    # Advance simulation
    manager.Update()
    system.DoStepDynamics(time_step)
    application.EndScene()
    current_time += time_step
    
    # Access and print camera data
    buffer = camera.GetMostRecentRGBA8Buffer()
    if buffer.HasData():
        rgba8_data = buffer.GetRGBA8Data()
        print(f"Time: {current_time:.2f}s | Camera Data: "
              f"First Pixel: {rgba8_data[0,0]} | "
              f"Buffer Shape: {rgba8_data.shape} | "
              f"Mean Intensity: {np.mean(rgba8_data):.1f}")

print("Simulation completed.")