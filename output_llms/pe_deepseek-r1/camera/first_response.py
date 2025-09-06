import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# Set path to Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))

# System and simulation parameters
sim_time = 0
time_step = 0.01
orbit_radius = 3.0
orbit_height = 1.0
orbit_speed = 0.5  # rad/s

# Create physical system
sys = chrono.ChSystemNSC()
sys.SetGravity(chrono.ChVector3d(0, -9.81, 0))

# Create ground plane
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True, ground_mat)
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, -1, 0))
ground.SetName("Ground")
sys.Add(ground)

# Create fixed mesh body (load from OBJ file)
mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.SetName("MeshBody")

# Add mesh visualization
mesh_asset = chrono.ChVisualShapeModelFile()
mesh_asset.SetFilename(chrono.GetChronoDataFile("models/cube.obj"))  # Replace with your OBJ file
mesh_asset.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
mesh_body.AddVisualShape(mesh_asset)
sys.Add(mesh_body)

# Create sensor manager
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3d(2, 2, 0), chrono.ChColor(1, 1, 1), 1000)

# Camera parameters
camera_res = (1280, 720)
camera_fov = 1.0  # Field of view in radians
camera_update_rate = 30  # Hz
camera_offset = chrono.ChVector3d(0, 0, 0)

# Create camera sensor
camera = sens.ChCameraSensor(
    mesh_body,              # Parent body
    camera_update_rate,     # Update rate
    chrono.ChFramed(        # Camera pose relative to body
        camera_offset,
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    ),
    camera_res[0],          # Image width
    camera_res[1],          # Image height
    camera_fov              # Field of view
)
camera.SetName("Orbit Camera")
camera.SetLag(0.1)  # Lag time for motion blur effect

# Add noise model
noise_model = sens.ChNoiseNormal(0.0, 0.1)  # Mean=0, stdev=0.1
camera.PushFilter(sens.ChFilterCameraNoise(noise_model))

# Add visualizations
camera.PushFilter(sens.ChFilterVisualize(camera_res[0], camera_res[1], "Camera View"))
camera.PushFilter(sens.ChFilterRGBA8Access())

# Add camera to manager
manager.AddSensor(camera)

# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Mesh with Orbiting Camera")
vis.Initialize()
vis.AddLogo()
vis.AddCamera(chrono.ChVector3d(0, 2, 4))
vis.AddTypicalLights()
vis.AddSkyBox()

# Main simulation loop
while vis.Run():
    # Update camera orbit position
    orbit_angle = orbit_speed * sim_time
    cam_x = orbit_radius * math.cos(orbit_angle)
    cam_z = orbit_radius * math.sin(orbit_angle)
    camera.SetOffsetPose(chrono.ChFramed(
        chrono.ChVector3d(cam_x, orbit_height, cam_z),
        chrono.Q_from_AngAxis(orbit_angle + math.pi, chrono.ChVector3d(0, 1, 0))
    ))

    # Advance simulation
    vis.BeginScene()
    vis.Render()
    manager.Update()
    sys.DoStepDynamics(time_step)
    vis.EndScene()
    
    # Access and print camera data
    if camera.GetMostRecentRGBA8Buffer():
        rgba8_buffer = camera.GetMostRecentRGBA8Buffer()
        rgba8_data = rgba8_buffer.GetRGBA8Data()
        print(f"Time: {sim_time:.2f}s | "
              f"Cam Pos: ({cam_x:.2f}, {orbit_height:.2f}, {cam_z:.2f}) | "
              f"First Pixel: {rgba8_data[0:3]}")
    
    sim_time += time_step