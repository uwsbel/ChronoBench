import pychrono.core as chrono
import pychrono.sensor as sens
import os

import math
import time

def main():
    # Create the physical system (non-smooth contact)
    mphysicalSystem = chrono.ChSystemNSC()

    # Load a triangular mesh from a Wavefront .obj file
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))  # scale by 2

    # Create a visual representation of the mesh
    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)  # immutable mesh

    # Create a fixed body and attach the visual mesh shape
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  # at origin
    mesh_body.AddVisualShape(trimesh_shape)  # attach visual shape
    mesh_body.SetFixed(True)  # fixed in space
    mphysicalSystem.Add(mesh_body)  # add to system

    # Create the sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # Add one point light (extra point lights at 9, 16, 23 removed)
    intensity = 1.0  # light intensity
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

    # Camera offset pose: changed from (-5, 0, 2) to (-7, 0, 2)
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 2), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))

    # Initialize the camera sensor with updated image dimensions (960x480)
    cam = sens.ChCameraSensor(
        mesh_body,              # body the camera is attached to
        update_rate,            # camera update rate in Hz
        offset_pose,            # offset pose of the camera
        image_width,            # image width in pixels (960)
        image_height,           # image height in pixels (480)
        fov                     # horizontal field of view in radians
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)  # lag between sensing and data accessibility
    cam.SetCollectionWindow(exposure_time)  # exposure time for the camera

    # Apply noise model based on specified type
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  # constant normal noise
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  # pixel-dependent noise
    elif noise_model == "NONE":
        pass  # no noise model

    # Visualize image before grayscale filter
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

    # Host access to RGBA8 buffer
    cam.PushFilter(sens.ChFilterRGBA8Access())

    # Save RGB images (save=True in turn 2)
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    # Convert to grayscale
    cam.PushFilter(sens.ChFilterGrayscale())

    # Visualize grayscale image
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    # Save grayscale images
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    # Resize image to half dimensions
    cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))

    # Host access to R8 buffer
    cam.PushFilter(sens.ChFilterR8Access())

    # Add the camera to the sensor manager
    manager.AddSensor(cam)

    # Simulation parameters
    orbit_radius = 10   # radius of the camera orbit
    orbit_rate = 0.1    # orbit rate changed from 0.5 to 0.1 radians per second
    ch_time = 0.0       # simulation time tracker


    t1 = time.time()  # record start time

    while ch_time < end_time:
        # Dynamically update camera position in orbit
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        # Access the RGBA8 buffer from the camera
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print("RGBA8 buffer received from cam. Camera resolution: {0}x{1}".format(rgba8_buffer.Width, rgba8_buffer.Height))
            print("First Pixel: {0}".format(rgba8_data[0, 0, :]))

        # Update sensor manager
        manager.Update()

        # Advance dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Update simulation time
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


# Camera parameters
noise_model = "CONST_NORMAL"  # constant normal noise model

# Update rate in Hz
update_rate = 30

# Image dimensions: changed from 1280x720 to 960x480
image_width = 960
image_height = 480

# Camera horizontal field of view
fov = 1.408  # horizontal FOV in radians

# Lag between sensing and accessibility
lag = 0

# Exposure time per image
exposure_time = 0

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 20.0

# Save camera images: changed from False to True
save = True

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/"

# Entry point
main()
