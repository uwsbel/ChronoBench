import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import os

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), '../../data/'))

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # No gravity for this simulation

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mesh with Orbiting Camera')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 1.5, 5), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)

# Load a mesh from a Wavefront .obj file (using the built-in Chrono mesh)
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('models/obj/torus.obj'), False, True)

# Create a body with the mesh
body = chrono.ChBody()
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetBodyFixed(True)  # Make the body fixed
body.GetVisualModel().AddTriangleMesh(mesh, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33<>(1), True)
system.Add(body)

# Create a camera sensor
camera = chrono.ChCameraSensor()
camera.SetName("orbiting_camera")
camera.SetCameraUp(chrono.ChVectorD(0, 0, 1))
camera.SetCameraLocation(chrono.ChVectorD(2, 0, 0))
camera.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
camera.SetFocalLength(0.035)  # 35mm focal length
camera.SetResolution(640, 480)
camera.SetFPS(30)
camera.SetAntiAliasing(chrono.ChCameraSensor::ANTIALIASING_4X)

# Add noise filters to the camera
camera.AddNoise(chrono.ChNoiseWhite(0.01))  # Small white noise
camera.AddNoise(chrono.ChNoiseGaussian(0.005))  # Small Gaussian noise

# Create a sensor manager
sensor_manager = chrono.ChSensorManager(system)
sensor_manager.AddSensor(camera)

# Add visualization of the camera images
vis.AddSensor(camera)
vis.EnableSensor(camera, True)

# Simulation parameters
time_step = 0.01
end_time = 10.0
output_step = 0.1

# Simulation loop
current_time = 0
while vis.Run() and current_time < end_time:
    # Update the camera position in an orbit around the mesh
    angle = current_time * 0.5  # Orbit speed
    radius = 2.0
    camera_x = radius * np.cos(angle)
    camera_y = radius * np.sin(angle)
    camera.SetCameraLocation(chrono.ChVectorD(camera_x, camera_y, 0.5))

    # Update the camera look-at point to always point at the mesh
    camera.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))

    # Advance the simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Print camera buffer data (simplified for this example)
    if current_time % output_step < time_step:
        print(f"Time: {current_time:.2f}s - Camera position: ({camera_x:.2f}, {camera_y:.2f}, 0.5)")

    # Advance the simulation time
    system.DoStepDynamics(time_step)
    current_time += time_step

    # Update the sensor manager
    sensor_manager.Update()