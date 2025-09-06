import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    # -----------------
    # Create the system
    # -----------------
    # Initialize the Chrono physical system using Non-Smooth Contact (NSC)
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------------------
    # Add a mesh to be sensed by a camera
    # -----------------------------------
    # Load a triangular mesh from a Wavefront .obj file
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    # Scale the mesh uniformly by a factor of 2
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

    # Create a visual representation of the mesh
    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)  # Set the mesh to be immutable

    # Create a body to which the visual shape will be attached
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  # Set the position of the body
    mesh_body.AddVisualShape(trimesh_shape)  # Attach the visual shape to the body
    mesh_body.SetFixed(True)  # Fix the body in space
    mphysicalSystem.Add(mesh_body)  # Add the body to the physical system

    # -----------------------
    # Create a sensor manager
    # -----------------------
    # Initializes the sensor manager to manage all sensors in the simulation
    manager = sens.ChSensorManager(mphysicalSystem)

    # Add point lights to the scene for illumination
    intensity = 1.0  # Set the light intensity
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0))

    # ------------------------------------------------
    # Create a filter graph for post-processing the data from the camera
    # ------------------------------------------------
    # Apply noise model to the camera sensor based on the specified type
    if noise_model == "CONST_NORMAL":
        cam = sens.ChCameraSensor(
            mesh_body,              # Body the camera is attached to
            update_rate,            # Camera update rate in Hz
            offset_pose,            # Offset pose of the camera
            image_width,            # Image width in pixels
            image_height,           # Image height in pixels
            fov                     # Camera's horizontal field of view in radians
        )
        cam.SetName("Camera Sensor")
        cam.SetLag(lag)  # Set the lag between sensing and when data becomes accessible
        cam.SetCollectionWindow(exposure_time)  # Set the exposure time for the camera

    # ------------------------------------------------------------------
    # Create a visual shape
    # --------------------------------------------------
    # Create a visual shape for the simulation
    #  This is a simple example, adjust the parameters to your needs.
    visual_shape = chrono.ChVisualShapeTriangleMesh()
    visual_shape.SetMesh(mmesh)
    visual_shape.SetName("HMMWV Chassis Mesh")
    visual_shape.SetMutable(False)  # Set the mesh to be immutable

    # ------------------------------------------------------------------
    # Create a body to which the visual shape will be attached
    # --------------------------------------------------
    # Create a body to which the visual shape will be attached
    body = chrono.ChBody()
    body.SetPos(chrono.ChVector3d(0, 0, 0))  # Set the position of the body
    body.AddVisualShape(visual_shape)  # Attach the visual shape to the body
    body.SetFixed(True)  # Fix the body in space
    mphysicalSystem.Add(body)  # Add the body to the physical system

    # ------------------------------------------------------------------
    # Create a sensor manager
    # --------------------------------------------------
    # Initialize the sensor manager to manage all sensors in the simulation
    manager = sens.ChSensorManager(mphysicalSystem)

    # Add point lights to the scene for illumination
    if vis:
        manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
        manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
        manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
        manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0))

    # ------------------------------------------------------------------
    # Create a filter graph for post-processing the data from the camera
    # --------------------------------------------------
    # Apply noise model to the camera sensor based on the specified type
    if noise_model == "CONST_NORMAL":
        cam = sens.ChCameraSensor(
            mesh_body,              # Body the camera is attached to
            update_rate,            # Camera update rate in Hz
            offset_pose,            # Offset pose of the camera
            image_width,            # Image width in pixels
            image_height,           # Image height in pixels
            fov                     # Camera's horizontal field of view in radians
        )
        cam.SetName("Camera Sensor")
        cam.SetLag(lag)  # Set the lag between sensing and when data becomes accessible
        cam.SetCollectionWindow(exposure_time)  # Set the exposure time for the camera

    # ------------------------------------------------------------------
    # Create a simulation loop
    # --------------------------------------------------
    # Execute the simulation
    #  The loop runs for a specified time step
    # --------------------------------------------------
    # Simulate system
    orbit_radius = 10  # Radius of the camera orbit
    orbit_rate = 0.5   # Rate of the camera orbit in radians per second
    ch_time = 0.0      # Initialize simulation time

    t1 = time.time()  # Record the start time of the simulation
    while ch_time < end_time:
        # Dynamically set the camera's position around the orbit
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1)))
        )

        # Access the RGBA8 buffer from the camera
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

        # Update the sensor manager (render/save/filter data automatically)
        manager.Update()

        # Perform a step of dynamics simulation
        mphysicalSystem.DoStepDynamics(step_size)

        # Update the current simulation time
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Camera parameters
# -----------------

# Noise model attached to the sensor
noise_model = "CONST_NORMAL"  # Constant normal noise model

# Camera lens model
lens_model = sens.PINHOLE  # Pinhole lens model

# Update rate in Hz
update_rate = 30

# Image width and height
image_width = 1280
image_height = 720

# Camera's horizontal field of view
fov = 1.408  # Horizontal field of view in radians

# Lag (in seconds) between sensing and when data becomes accessible
lag = 0

# Exposure (in seconds) of each image
exposure_time = 0

# ------------------------------------------------------------------
# Pipeline for Generating New Simulations
# --------------------------------------------------

# Generate the Initial Code Snippet
# --------------------------------------------------
# Initialize the Chrono physical system using Non-Smooth Contact (NSC)
mphysicalSystem = chrono.ChSystemNSC()

# -----------------------------------
# Add a mesh to be sensed by a camera
# -----------------------------------
# Load a triangular mesh from a Wavefront .obj file
mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
# Scale the mesh uniformly by a factor of 2
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

# Create a visual representation of the mesh
trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("HMMWV Chassis Mesh")
trimesh_shape.SetMutable(False)  # Set the mesh to be immutable

# Create a body to which the visual shape will be attached
mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  # Set the position of the body
mesh_body.AddVisualShape(trimesh_shape)  # Attach the visual shape to the body
mesh_body.SetFixed(True)  # Fix the body in space
mphysicalSystem.Add(mesh_body)  # Add the body to the physical system

# -----------------------------------
# Create a sensor manager
# -----------------------------------
# Initializes the sensor manager to manage all sensors in the simulation
manager = sens.ChSensorManager(mphysicalSystem)

# Add point lights to the scene for illumination
if vis:
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0))

# -----------------------------------
# Create a filter graph for post-processing the data from the camera
# -----------------------------------
# Apply noise model to the camera sensor based on the specified type
if noise_model == "CONST_NORMAL":
    cam = sens.ChCameraSensor(
        mesh_body,              # Body the camera is attached to
        update_rate,            # Camera update rate in Hz
        offset_pose,            # Offset pose of the camera
        image_width,            # Image width in pixels
        image_height,           # Image height in pixels
        fov                     # Camera's horizontal field of view in radians
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)  # Set the lag between sensing and when data becomes accessible
    cam.SetCollectionWindow(exposure_time)  # Set the exposure time for the camera

# -----------------------------------
# Create a visual shape
# -----------------------------------
# Create a visual shape for the simulation
#  This is a simple example, adjust the parameters to your needs.
visual_shape = chrono.ChVisualShapeTriangleMesh()
visual_shape.SetMesh(mmesh)
visual_shape.SetName("HMMWV Chassis Mesh")
visual_shape.SetMutable(False)  # Set the mesh to be immutable

# -----------------------------------
# Create a body to which the visual shape will be attached
# -----------------------------------
# Create a body to which the visual shape will be attached
# -----------------------------------
# Create a sensor manager
# -----------------------------------
# Initializes the sensor manager to manage all sensors in the simulation
manager = sens.ChSensorManager(mphysicalSystem)

# Add point lights to the scene for illumination
if vis:
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0))

# -----------------------------------
# Simulate system
# -----------------------------------
# Simulate system
#  The loop runs for a specified time step
# -----------------------------------
# Simulate system
orbit_radius = 10  # Radius of the camera orbit
orbit_rate = 0.5   # Rate of the camera orbit in radians per second
ch_time = 0.0      # Initialize simulation time

t1 = time.time()  # Record the start time of the simulation
while ch_time < end_time:
    # Dynamically set the camera's position around the orbit
    cam.SetOffsetPose(chrono.ChFramed(
        chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
        chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1)))
    )

    # Access the RGBA8 buffer from the camera
    rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
    if rgba8_buffer.HasData():
        rgba8_data = rgba8_buffer.GetRGBA8Data()
        print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
        print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

    # Update the sensor manager (render/save/filter data automatically)
    manager.Update()

    # Perform a step of dynamics simulation
    mphysicalSystem.DoStepDynamics(step_size)

    # Update the current simulation time
    ch_time = mphysicalSystem.GetChTime()

# -----------------
# Camera parameters
# -----------------

# Noise model attached to the sensor
noise_model = "CONST_NORMAL"  # Constant normal noise model

# Camera lens model
lens_model = sens.PINHOLE  # Pinhole lens model

# Update rate in Hz
update_rate = 30

# Image width and height
image_width = 1280
image_height = 720

# Camera's horizontal field of view
fov = 1.408  # Horizontal field of view in radians

# Lag (in seconds) between sensing and when data becomes accessible
lag = 0

# Exposure (in seconds) of each image
exposure_time = 0

# -----------------------------------
# Pipeline for Generating New Simulations
# -----------------------------------

# Generate the Initial Code Snippet
# --------------------------------------------------
# Initialize the Chrono physical system using Non-Smooth Contact (NSC)
mphysicalSystem = chrono.ChSystemNSC()

# -----------------------------------
# Add a mesh to be sensed by a camera
# -----------------------------------
# Load a triangular mesh from a Wavefront .obj file
mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
# Scale the mesh uniformly by a factor of 2
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

# Create a visual representation of the mesh
trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("HMMWV Chassis Mesh")
trimesh_shape.SetMutable(False)  # Set the mesh to be immutable

# Create a body to which the visual shape will be attached
mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  # Set the position of the body
mesh_body.AddVisualShape(trimesh_shape)  # Attach the visual shape to the body
mesh_body.SetFixed(True)  # Fix the body in space
mphysicalSystem.Add(mesh_body)  # Add the body to the physical system

# -----------------------------------
# Create a sensor manager
# -----------------------------------
# Initializes the sensor manager to manage all sensors in the simulation
manager = sens.ChSensorManager(mphysicalSystem)

# Add point lights to the scene for illumination
if vis:
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0))

# -----------------------------------
# Create a filter graph for post-processing the data from the camera
# -----------------------------------
# Apply noise model to the camera sensor based on the specified type
if noise_model == "CONST_NORMAL":
    cam = sens.ChCameraSensor(
        mesh_body,              # Body the camera is attached to
        update_rate,            # Camera update rate in Hz
        offset_pose,            # Offset pose of the camera
        image_width,            # Image width in pixels
        image_height,           # Image height in pixels
        fov                     # Camera's horizontal field of view in radians
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)  # Set the lag between sensing and when data becomes accessible
    cam.SetCollectionWindow(exposure_time)  # Set the exposure time for the camera

# -----------------------------------
# Create a visual shape
# -----------------------------------
# Create a visual shape for the simulation
#  This is a simple example, adjust the parameters to your needs.
visual_shape = chrono.ChVisualShapeTriangleMesh()
visual_shape.SetMesh(mmesh)
visual_shape.SetName("HMMWV Chassis Mesh")
visual_shape.SetMutable(False)  # Set the mesh to be immutable

# -----------------------------------
# Create a body to which the visual shape will be attached
# -----------------------------------
# Create a body to which the visual shape will be attached
# -----------------------------------
# Create a sensor manager
# -----------------------------------
# Initializes the sensor manager to manage all sensors in the simulation
manager = sens.ChSensorManager(mphysicalSystem)

# Add point lights to the scene for illumination
if vis:
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0))

# -----------------------------------
# Simulate system
# -----------------------------------
# Simulate system
#  The loop runs for a specified time step
# -----------------------------------
# Simulate system
orbit_radius = 10  # Radius of the camera orbit
orbit_rate = 0.5   # Rate of the camera orbit in radians per second
ch_time = 0.0      # Initialize simulation time

t1 = time.time()  # Record the start time of the simulation
while ch_time < end_time:
    # Dynamically set the camera's position around the orbit
    cam.SetOffsetPose(chrono.ChFramed(
        chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
        chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1)))
    )

    # Access the RGBA8 buffer from the camera
    rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
    if rgba8_buffer.HasData():
        rgba8_data = rgba8_buffer.GetRGBA8Data()
        print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
        print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

    # Update the sensor manager (render/save/filter data automatically)
    manager.Update()

    # Perform a step of dynamics simulation
    mphysicalSystem.DoStepDynamics(step_size)

    # Update the current simulation time
    ch_time = mphysicalSystem.GetChTime()

# -----------------
# Camera parameters
# -----------------

# Noise model attached to the sensor
noise_model = "CONST_NORMAL"  # Constant normal noise model

# Camera lens model
lens_model = sens.PINHOLE  # Pinhole lens model

# Update rate in Hz
update_rate = 30

# Image width and height
image_width = 1280
image_height = 720

# Camera's horizontal field of view
fov = 1.408  # Horizontal field of view in radians

# Lag (in seconds) between sensing and when data becomes accessible
lag = 0

# Exposure (in seconds) of each image
exposure_time = 0

# -----------------------------------
# Pipeline for Generating New Simulations
# -----------------------------------

# Generate the Initial Code Snippet
# --------------------------------------------------
# Initialize the Chrono physical system using Non-Smooth Contact (NSC)
mphysicalSystem = chrono.ChSystemNSC()

# -----------------------------------
# Add a mesh to be sensed by a camera
# -----------------------------------
# Load a triangular mesh from a Wavefront .obj file
mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
# Scale the mesh uniformly by a factor of 2
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

# Create a visual representation of the mesh
trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("HMMWV Chassis Mesh")
trimesh_shape.SetMutable(False)  # Set the mesh to be immutable

# Create a body to which the visual shape will be attached
mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  # Set the position of the body
mesh_body.AddVisualShape(trimesh_shape)  # Attach the visual shape to the body
mesh_body.SetFixed(True)  # Fix the body in space
mphysicalSystem.Add(mesh_body)  # Add the body to the physical system

# -----------------------------------
# Create a sensor manager
# -----------------------------------
# Initializes the sensor manager to manage all sensors in the simulation
manager = sens.ChSensorManager(mphysicalSystem)

# Add point lights to the scene for illumination
if vis:
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0))

# -----------------------------------
# Create a filter graph for post-processing the data from the camera
# -----------------------------------
# Apply noise model to the camera sensor based on the specified type
if noise_model == "CONST_NORMAL":
    cam = sens.ChCameraSensor(
        mesh_body,              # Body the camera is attached to
        update_rate,            # Camera update rate in Hz
        offset_pose,            # Offset pose of the camera
        image_width,            # Image width in pixels
        image_height,           # Image height in pixels
        fov                     # Camera's horizontal field of view in radians
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)  # Set the lag between sensing and when data becomes accessible
    cam.SetCollectionWindow(exposure_time)  # Set the exposure time for the camera

# -----------------------------------
# Create a visual shape
# -----------------------------------
# Create a visual shape for the simulation
#  This is a simple example, adjust the parameters to your needs.
visual_shape = chrono.ChVisualShapeTriangleMesh()
visual_shape.SetMesh(mmesh)
visual_shape.SetName("HMMWV Chassis Mesh")
visual_shape.SetMutable(False)  # Set the mesh to be immutable

# -----------------------------------
# Create a body to which the visual shape will be attached
# -----------------------------------
# Create a body to which the visual shape will be attached
# -----------------------------------
# Create a sensor manager
# -----------------------------------
# Initializes the sensor manager to manage all sensors in the simulation
manager = sens.ChSensorManager(mphysicalSystem)

# Add point lights to the scene for illumination
if vis:
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0))