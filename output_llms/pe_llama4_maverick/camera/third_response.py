import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time
import os

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------------------
    # Add a box to be sensed by a camera
    # -----------------------------------
    side = 2  # Define the side length of the box
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000, True, True)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)  # Fix the box in space
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # Add point lights to the scene
    intensity = 1.0
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 3), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
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

    # ------------------------------------------------------------------
    # Create a filter graph for post-processing the data from the camera
    # ------------------------------------------------------------------
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
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
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

# -----------------
# Camera parameters
# -----------------
noise_model = "CONST_NORMAL"
update_rate = 30
image_width = 1280
image_height = 720
fov = 1.408
lag = 0
exposure_time = 0

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 20.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"

# Main function entry point
main()