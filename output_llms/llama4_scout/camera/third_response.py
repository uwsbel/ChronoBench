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
    # Add a box to be sensed by a camera 
    # ----------------------- 
    side = 2.0
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)

    # Create a texture
    texture = chrono.ChTexture()
    texture.LoadFromFile("textures/skin.jpg")

    # Add texture to the box
    box_shape = box_body.GetVisualShape(0)
    box_shape.SetTexture(texture)

    mphysicalSystem.Add(box_body)

    # ----------------------- 
    # Create a sensor manager 
    # ----------------------- 
    # Initialize the sensor manager to manage all sensors in the simulation
    manager = sens.ChSensorManager(mphysicalSystem)

    # Add point lights to the scene for illumination
    intensity = 1.0  
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

    # ------------------------------------------------ 
    # Create a camera and add it to the sensor manager 
    # ------------------------------------------------ 
    # Define the camera offset pose relative to the body it is attached to
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 3), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))

    # Camera parameters
    update_rate = 30  
    image_width = 1280 
    image_height = 720 
    fov = 1.408 
    lag = 0 
    exposure_time = 0 

    # Initialize the camera sensor
    cam = sens.ChCameraSensor(
        box_body,  
        update_rate,  
        offset_pose,  
        image_width,  
        image_height,  
        fov  
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag) 
    cam.SetCollectionWindow(exposure_time) 

    # Noise model attached to the sensor
    noise_model = "CONST_NORMAL" 
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02)) 
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03)) 
    elif noise_model == "NONE":
        pass 

    # Visualize the image before applying grayscale filter
    vis = True
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

    # Provide host access to the RGBA8 buffer from the camera
    cam.PushFilter(sens.ChFilterRGBA8Access())

    # Save camera images
    save = False
    out_dir = "SENSOR_OUTPUT/"
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    # Convert the camera image to grayscale
    cam.PushFilter(sens.ChFilterGrayscale())

    # Visualize the grayscaled image
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    # Save the grayscaled image
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    # Resize the image
    cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))

    # Access the grayscaled image buffer as R8 pixels
    cam.PushFilter(sens.ChFilterR8Access())

    # Add the camera sensor to the manager
    manager.AddSensor(cam)

    # --------------- 
    # Simulate system 
    # --------------- 
    orbit_radius = 10 
    orbit_rate = 0.5 
    ch_time = 0.0 
    end_time = 20.0
    step_size = 1e-3

    t1 = time.time() 
    while ch_time < end_time: 
        # Dynamically set the camera's position around the orbit
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        ))

        # Access the RGBA8 buffer from the camera
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

        # Update the sensor manager 
        manager.Update()

        # Perform a step of dynamics simulation
        mphysicalSystem.DoStepDynamics(step_size)

        # Update the current simulation time
        ch_time = mphysicalSystem.GetChTime()
    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()