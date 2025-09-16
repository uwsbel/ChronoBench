import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# --- Create the simulation system ---
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# --- Create the triangular mesh body ---
mesh_file = "cube.obj"  # Replace with your .obj file
mesh_body = chrono.ChBodyEasy()
mesh_body.SetBodyFixed(True)
mesh_body.AddAsset(chrono.ChTriangleMeshConnected())
mesh_body.GetAsset().LoadFromFile(mesh_file)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(mesh_body)

# --- Create a camera sensor ---
camera_sensor = chrono.ChSensorCamera()
camera_sensor.Setup(mesh_body, chrono.ChVectorD(0, 0, -1),  # Pointing direction
                    640, 480,  # Image resolution
                    0.1, 100,  # Near and far clipping planes
                    60)  # Field of view (degrees)

# --- Add noise filters to the camera ---
noise_filter = chrono.ChSensorNoiseGaussian()
noise_filter.SetNoiseLevel(0.01)  # Adjust noise level as needed
camera_sensor.AddFilter(noise_filter)

# --- Create a sensor manager ---
sensor_manager = chrono.ChSensorManager()
sensor_manager.AddSensor(camera_sensor)
system.Add(sensor_manager)

# --- Visualization settings ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Camera Sensor Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5))  # Initial camera position
vis.AddTypicalLights()

# --- Orbiting camera ---
orbit_radius = 3.0
orbit_speed = 0.1
angle = 0.0

# --- Simulation loop ---
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # --- Update camera position in orbit ---
    angle += orbit_speed
    camera_pos = chrono.ChVectorD(orbit_radius * np.cos(angle), 2, orbit_radius * np.sin(angle))
    vis.GetIrrlichtApplication().GetSceneManager().GetActiveCamera().SetPosition(camera_pos)
    vis.GetIrrlichtApplication().GetSceneManager().GetActiveCamera().LookAt(chrono.ChVectorD(0,0,0))

    # --- Get camera buffer data ---
    if sensor_manager.GetSensorCount() > 0:
        camera_data = sensor_manager.GetSensorData(0)
        if camera_data is not None:
            # Print some basic info about the camera data
            print(f"Timestamp: {camera_data.timestamp}")
            print(f"Image width: {camera_data.width}")
            print(f"Image height: {camera_data.height}")

            # Access the image data (as a numpy array)
            image_data = camera_data.image
            # You can further process or save the image data here
            # For example, save as a PNG image:
            # import imageio
            # imageio.imwrite("camera_output.png", image_data)
    
    system.DoStepDynamics(0.01)