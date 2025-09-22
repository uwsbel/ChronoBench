import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time
import os # Added for creating output directory

def main():
    # -----------------
    # Camera parameters (moved inside main)
    # -----------------
    # Noise model attached to the sensor
    noise_model = "CONST_NORMAL"  # Constant normal noise model

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

    # ---------------------
    # Simulation parameters (moved inside main)
    # ---------------------
    # Simulation step size
    step_size = 1e-3

    # Simulation end time
    end_time = 20.0

    # Save camera images
    save = False

    # Render camera images
    vis = True

    # Output directory
    out_dir = "SENSOR_OUTPUT/"
    if save:
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
        if not os.path.exists(out_dir + "rgb/"):
            os.makedirs(out_dir + "rgb/")
        if not os.path.exists(out_dir + "gray/"):
            os.makedirs(out_dir + "gray/")


    # Box dimension (as per new requirement)
    side = 1.0 

    # -----------------
    # Create the system
    # -----------------
    # Initialize the Chrono physical system (non-smooth contact)
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------------------
    # Add a box to be sensed by a camera (MODIFIED SECTION)
    # -----------------------------------
    # Create a ChBodyEasyBox
    # Parameters: half_x, half_y, half_z, density, visualize_asset, collide
    box_body = chrono.ChBodyEasyBox(side / 2, side / 2, side / 2,  # half dimensions
                                     1000,                          # density
                                     True,                          # create visualization asset
                                     True)                          # enable collision
    
    # Set position of the box (center of base at Z=0, so centroid at Z=side/2)
    box_body.SetPos(chrono.ChVector3d(0, 0, side / 2))
    box_body.SetFixed(True)  # Fix the body in space
    mphysicalSystem.Add(box_body)  # Add the body to the physical system

    # Add a texture to the box
    # ChBodyEasyBox creates a ChVisualShape with a default ChVisualMaterial.
    # We get this visual shape and modify its material.
    if len(box_body.GetVisualShapes()) > 0:
        visual_shape = box_body.GetVisualShape(0) # Gets the ChVisualShape associated with the box
        if visual_shape:
            # ChVisualShape contains a list of materials. ChBodyEasyBox adds one by default.
            if visual_shape.material_list and len(visual_shape.material_list) > 0:
                # Modify the existing material
                visual_shape.material_list[0].SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
            else:
                # Or, if no material somehow (should not happen for EasyBox), add a new one
                custom_material = chrono.ChVisualMaterial()
                custom_material.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
                visual_shape.material_list.append(custom_material)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    # Initialize the sensor manager to manage all sensors in the simulation
    manager = sens.ChSensorManager(mphysicalSystem)

    # Add point lights to the scene for illumination
    intensity = 1.0  # Set the light intensity
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    # Area light position (0,0,4) should still illuminate the box (at 0,0,side/2) from above.
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, 
                               chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0)) # Shines in -Z direction

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    # Define the camera offset pose relative to the body it is attached to
    # Original offset was chrono.ChVector3d(-5, 0, 2), original rotation was QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0))
    # New position part: chrono.ChVector3d(-7, 0, 3). Rotation part remains the same.
    original_rotation_quat = chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0))
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 3), original_rotation_quat)

    # Initialize the camera sensor
    cam = sens.ChCameraSensor(
        box_body,               # MODIFIED: Camera attached to box_body
        update_rate,            # Camera update rate in Hz
        offset_pose,            # MODIFIED: Offset pose of the camera
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
    # Apply noise model to the camera sensor based on the specified type
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  # Add constant normal noise
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  # Add pixel-dependent noise
    elif noise_model == "NONE":
        # No noise model applied
        pass

    # Visualize the image before applying grayscale filter
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

    # Provide host access to the RGBA8 buffer from the camera
    cam.PushFilter(sens.ChFilterRGBA8Access())

    # Save the current image to a PNG file at the specified path
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
    orbit_radius = 10  # Radius of the camera orbit (relative to box_body)
    orbit_rate = 0.5   # Rate of the camera orbit in radians per second
    ch_time = 0.0      # Initialize simulation time

    t1 = time.time()  # Record the start time of the simulation

    while ch_time < end_time:
        # Dynamically set the camera's pose relative to the box_body (orbiting the box_body)
        # The camera's Z-axis (viewing direction) will be parallel to the box_body's Z-axis.
        # To make it look at the box_body's origin, the quaternion would need further adjustment.
        # Keeping original orbit behavior's orientation logic:
        cam_orbit_pos = chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate),
                                          -orbit_radius * math.sin(ch_time * orbit_rate),
                                          1) # Z=1 relative to box_body's origin
        cam_orbit_rot = chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        
        cam.SetOffsetPose(chrono.ChFramed(cam_orbit_pos, cam_orbit_rot))

        # Access the RGBA8 buffer from the camera
        rgba8_buffer_ptr = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer_ptr.HasData():
            # Get the ChPixelAccess object for reading pixel data
            pixel_data_accessor = rgba8_buffer_ptr.GetRGBA8Data() 
            print(f'RGBA8 buffer received from cam. Camera resolution: {rgba8_buffer_ptr.Width}x{rgba8_buffer_ptr.Height}')
            # Accessing pixel data: ChPixelAccess[row, col] returns a list [R, G, B, A]
            if rgba8_buffer_ptr.Width > 0 and rgba8_buffer_ptr.Height > 0:
                first_pixel_rgba = pixel_data_accessor[0,0] 
                print(f'First Pixel (RGBA): {first_pixel_rgba}')

        # Update the sensor manager (render/save/filter data automatically)
        manager.Update()

        # Perform a step of dynamics simulation
        mphysicalSystem.DoStepDynamics(step_size)

        # Update the current simulation time
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# Main function entry point
if __name__ == '__main__':
    main()