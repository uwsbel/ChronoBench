import os
import math
import numpy as np
import pychrono as chrono
import pychrono.sensor as sens

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Define properties for the triangle mesh
mesh_scale = chrono.ChVector3d(3, 3, 3)  # Scale the mesh
triang_mesh = chrono.ChTriangleMeshConnected()
# Load the mesh from a Wavefront OBJ file
triang_mesh.LoadWavefrontMesh(
    chrono.GetChronoDataFile("gripper/wrist_triangles.obj"), False, True)
# Transform the mesh: scale and center it
triang_mesh.Transform(
    mesh_scale, chrono.ChVector3d(-1.8, 0, -1.5))

# Create a visual object for the mesh
mesh_shape = chrono.ChVisualShapeTriangleMesh()
mesh_shape.SetMesh(triang_mesh)
mesh_shape.SetName("Wrist")
mesh_shape.SetMutable(False)

# Create a body and add the visual shape to it
body = chrono.ChBody()
body.AddVisualShape(mesh_shape)
body.SetFixed(True)  # Fix the body in space
sys.AddBody(body)  # Add the body to the simulation system

# Initialize the sensor manager
manager = sens.ChSensorManager(sys)

# Create and configure a camera sensor
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 1, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
camera = sens.ChCameraSensor(
    body,              # Body to which the camera is attached
    30,                # Update rate in Hz
    offset_pose,       # Offset pose of the camera
    1280, 720,          # Image resolution
    1.5708,            # Field of view (in radians)
    1,                 # Lag time
    sens.IMAGE_PIXELS, # Camera output type
    3.8                 # Exposure time
)
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Frustum buffer"))
camera.PushFilter(sens.ChFilterRGBA8ToIntensity())
camera.PushFilter(sens.ChFilterObservationToTexture())
manager.AddSensor(camera)  # Add the camera to the sensor manager

# Create a second camera with different configurations
offset_pose2 = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
camera2 = sens.ChCameraSensor(
    body,              # Body to which the camera is attached
    30,                # Update rate in Hz
    offset_pose2,      # Offset pose of the camera
    1280, 720,          # Image resolution
    1.5708,            # Field of view (in radians)
    1,                 # Lag time
    sens.IMAGE_INTENSITY, # Camera output type
    3.8                 # Exposure time
)
camera2.PushFilter(sens.ChFilterVisualize(1280, 720, "Gray buffer"))
camera2.PushFilter(sens.ChFilterDenoising())
camera2.PushFilter(sens.ChFilterMean(5))
camera2.PushFilter(sens.ChFilterR recentToIntensity())
manager.AddSensor(camera2)  # Add the second camera to the sensor manager

# Set the simulation time step and run time
step_size = 1e-3
run_time = 100

# Initialize the simulation
sys.Setup()

# Simulation loop
rot = 0
counter = 0
while counter < 2 * run_time:
    # Orbit the camera around the scene
    cam_offset = chrono.ChFramed(chrono.ChVector3d(3 * math.cos(rot), 2, 3 * math.sin(rot)), chrono.QuatFromAngleAxis(rot, chrono.ChVector3d(0, 1, 0)))
    camera.SetOffsetPose(cam_offset)
    
    # Print the current intensity buffer from the camera
    print("Image buffer out ", counter, " ", camera.GetMostRecentIntensityBuffer())
    
    # Update the sensor manager
    manager.Update()
    
    # Advance the simulation by one step
    sys.DoStepDynamics(step_size)
    rot += step_size / 2  # Increment the rotation angle
    counter += 1  # Increment the counter