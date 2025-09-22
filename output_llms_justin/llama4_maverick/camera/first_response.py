import pychrono as chrono
import pychrono.sensor as sens
import numpy as np

# 1. Initialize the PyChrono environment and core components.
chrono_system = chrono.ChSystemNSC()
chrono_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Set gravitational acceleration to zero

# 2. Add the required physical systems and objects as specified.
# Load a triangular mesh from a Wavefront .obj file
mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/forklift/meshes/forklift_body.obj"))
mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))  # Optional transformation

# Create a fixed body with the mesh
body = chrono.ChBody()
body.SetBodyFixed(True)  # Make the body fixed
body_shape = chrono.ChTriangleMeshShape()
body_shape.SetMesh(mesh)
body.AddVisualShape(body_shape)
chrono_system.Add(body)

# 3. Set necessary default parameters such as positions, forces, and interactions.
# Initialize the body position
body.SetPos(chrono.ChVectorD(0, 0, 0))

# Create a camera sensor
offset_pose = chrono.ChFrameD(chrono.ChVectorD(-5, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
camera = sens.ChCameraSensor(
    body, 
    30,  # update rate
    offset_pose,
    640,  # image width
    480,  # image height
    chrono.ChFrad(chrono.CH_C_PI / 4)  # FOV
)
camera.SetName("Camera Sensor")
camera.PushFilter(sens.ChFilterRGBA8())
camera.PushFilter(sens.ChFilterGrayscale())
camera.PushFilter(sens.ChFilterNoiseSaltPepper(0.1))  # Add salt and pepper noise
camera.PushFilter(sens.ChFilterVisualize(640, 480, "Camera Output"))

# Add the camera to the sensor manager
manager = sens.ChSensorManager(chrono_system)
manager.AddSensor(camera)

# Simulation loop
time_step = 1e-3
time_end = 10
for time in np.arange(0, time_end, time_step):
    # Update the camera's position in an orbit around the mesh
    orbit_radius = 5
    camera_angle = time * 0.1  # Adjust this to change the orbit speed
    camera_pos = chrono.ChVectorD(orbit_radius * np.cos(camera_angle), 0, orbit_radius * np.sin(camera_angle))
    offset_pose = chrono.ChFrameD(camera_pos, chrono.Q_from_AngAxis(camera_angle, chrono.ChVectorD(0, 1, 0)))
    camera.SetOffsetPose(offset_pose)
    
    # Simulate the system
    chrono_system.Update()
    manager.Update()
    
    # Access and print camera buffer data
    buffer = camera.GetMostRecentBuffer()
    if buffer.HasData():
        print(f"Time: {time}, Camera Buffer Data (grayscale): {buffer.GetRGBA8().shape}")

    # Step the system
    chrono_system.DoStepDynamics(time_step)

# Execution example
if __name__ == "__main__":
    pass  # The script is designed to run as is.