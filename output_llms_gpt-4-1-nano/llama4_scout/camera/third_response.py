import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

# -----------------
# Camera parameters
# -----------------
noise_model = "CONST_NORMAL"  # Constant normal noise model
update_rate = 30  # Update rate in Hz
image_width = 1280  # Image width
image_height = 720  # Image height
fov = 1.408  # Camera's horizontal field of view in radians
lag = 0  # Lag (in seconds) between sensing and when data becomes accessible
exposure_time = 0  # Exposure (in seconds) of each image

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3  # Simulation step size
end_time = 20.0  # Simulation end time
save = False  # Save camera images
vis = True  # Render camera images
out_dir = "SENSOR_OUTPUT/"  # Output directory
side = 2  # Side length of the box

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------------------
    # Add a box to be sensed by a camera
    # -----------------------------------
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))  # Set the position of the body
    box_body.SetFixed(True)  # Fix the body in space
    mphysicalSystem.Add(box_body)  # Add the body to the physical system

    # -----------------------
    # Create a sensor manager
    # -----------------------
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
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 3), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))

    cam = sens.ChCameraSensor(
        box_body,              # Body the camera is attached to
        update_rate,            # Camera update rate in Hz
        offset_pose,            # Offset pose of the camera
        image_width,            # Image width in pixels
        image_height,           # Image height in pixels
        fov                     # Camera's horizontal field of view in radians
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)  
    cam.SetCollectionWindow(exposure_time)  

    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  
    elif noise_model == "NONE":
        pass

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

    cam.PushFilter(sens.ChFilterRGBA8Access())

    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    cam.PushFilter(sens.ChFilterGrayscale())

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))

    cam.PushFilter(sens.ChFilterR8Access())

    manager.AddSensor(cam)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10  
    orbit_rate = 0.5   
    ch_time = 0.0      

    t1 = time.time()  

    while ch_time < end_time:
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

        manager.Update()

        mphysicalSystem.DoStepDynamics(step_size)

        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()