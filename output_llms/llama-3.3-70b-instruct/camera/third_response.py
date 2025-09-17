import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    # -----------------
    # Create the system
    # -----------------
    # Initialize the Chrono physical system (non-smooth contact)
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------
    # Create a box object
    # -----------------------
    # Define the side length of the box
    side = 1.0
    
    # Create a box object
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    
    # Set the position of the box
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))
    
    # Add the box to the physical system
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    # Initialize the sensor manager to manage all sensors in the simulation
    manager = sens.ChSensorManager(mphysicalSystem)

    # Add point lights to the scene for illumination
    intensity = 1.0  # Set the light intensity
    manager.scene.AddPointLight(chrono.ChVectorD(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorD(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorD(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorD(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVectorD(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, -1, 0))

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    # Define the camera parameters
    update_rate = 30  # Camera update rate in Hz
    image_width = 1280  # Image width in pixels
    image_height = 720  # Image height in pixels
    fov = 1.408  # Camera's horizontal field of view in radians
    lag = 0  # Lag (in seconds) between sensing and when data becomes accessible
    exposure_time = 0  # Exposure (in seconds) of each image

    # Define the camera offset pose relative to the body it is attached to
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-7, 0, 3), chrono.Q_from_AngX(2))

    # Initialize the camera sensor
    cam = sens.ChCameraSensor(
        box_body,              # Body the camera is attached to
        update_rate,            # Camera update rate in Hz
        offset_pose,            # Offset pose of the camera
        image_width,            # Image width in pixels
        image_height,           # Image height in pixels
        fov                     # Camera's horizontal field of view in radians
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)  # Set the lag between sensing and data accessibility
    cam.SetCollectionWindow(exposure_time)  # Set the exposure time for the camera

    # ------------------------------------------------------------------
    # Create a filter graph for post-processing the data from the camera
    # ------------------------------------------------------------------
    # Define the noise model
    noise_model = "CONST_NORMAL"  # Constant normal noise model

    # Apply noise model to the camera sensor based on the specified type
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  # Add constant normal noise
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  # Add pixel-dependent noise
    elif noise_model == "NONE":
        # No noise model applied
        pass

    # Visualize the image before applying grayscale filter
    vis = True
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

    # Provide host access to the RGBA8 buffer from the camera
    cam.PushFilter(sens.ChFilterRGBA8Access())

    # Save the current image to a PNG file at the specified path
    save = False
    out_dir = "SENSOR_OUTPUT/"
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    # Convert the camera image to grayscale
    cam.PushFilter(sens.ChFilterGrayscale())

    # Visualize the grayscaled image
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    # Save the grayscaled image to a PNG file at the specified path
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    # Resize the image to the specified width and height
    cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))

    # Access the grayscaled image buffer as R8 pixels
    cam.PushFilter(sens.ChFilterR8Access())

    # Add the camera sensor to the manager
    manager.AddSensor(cam)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10  # Radius of the camera orbit
    orbit_rate = 0.5   # Rate of the camera orbit in radians per second
    ch_time = 0.0      # Initialize simulation time
    step_size = 1e-3   # Simulation step size
    end_time = 20.0    # Simulation end time

    t1 = time.time()  # Record the start time of the simulation

    while ch_time < end_time:
        # Dynamically set the camera's position around the orbit
        cam.SetOffsetPose(chrono.ChFrameD(
            chrono.ChVectorD(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.Q_from_AngX(ch_time * orbit_rate)))

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

# Main function entry point
main()