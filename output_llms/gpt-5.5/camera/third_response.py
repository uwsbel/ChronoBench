import pychrono.core as chrono
import pychrono.sensor as sens

import math
import os
import time


def main():
    # -----------------
    # Create the system
    # -----------------
    # Initialize the Chrono physical system (non-smooth contact)
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------
    # Add a box to be sensed
    # -----------------------
    # Create a box body with dimensions side x side x side and density 1000
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)

    # Set a texture for the box visual shape
    box_body.GetVisualShape(0).SetTexture(
        chrono.GetChronoDataFile("textures/checker2.png")
    )

    # Add the box body to the physical system
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # Add lights to the scene for illumination
    intensity = 1.0
    manager.scene.AddPointLight(
        chrono.ChVector3f(2, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0,
    )
    manager.scene.AddPointLight(
        chrono.ChVector3f(9, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0,
    )
    manager.scene.AddPointLight(
        chrono.ChVector3f(16, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0,
    )
    manager.scene.AddPointLight(
        chrono.ChVector3f(23, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0,
    )
    manager.scene.AddAreaLight(
        chrono.ChVector3f(0, 0, 4),
        chrono.ChColor(intensity, intensity, intensity),
        500.0,
        chrono.ChVector3f(1, 0, 0),
        chrono.ChVector3f(0, -1, 0),
    )

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    # Camera offset pose relative to the body it is attached to
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-7, 0, 3),
        chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)),
    )

    # Initialize the camera sensor attached to the box
    cam = sens.ChCameraSensor(
        box_body,        # Body the camera is attached to
        update_rate,     # Camera update rate in Hz
        offset_pose,     # Offset pose of the camera
        image_width,     # Image width in pixels
        image_height,    # Image height in pixels
        fov,             # Horizontal field of view in radians
    )

    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    # ------------------------------------------------------------------
    # Create a filter graph for post-processing camera data
    # ------------------------------------------------------------------
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))
    elif noise_model == "NONE":
        pass
    else:
        raise ValueError(f"Unsupported noise model: {noise_model}")

    # Visualize the image before grayscale conversion
    if vis:
        cam.PushFilter(
            sens.ChFilterVisualize(
                image_width,
                image_height,
                "Before Grayscale Filter",
            )
        )

    # Provide host access to the RGBA8 buffer
    cam.PushFilter(sens.ChFilterRGBA8Access())

    # Save RGB images
    if save:
        rgb_dir = os.path.join(out_dir, "rgb") + os.sep
        gray_dir = os.path.join(out_dir, "gray") + os.sep
        os.makedirs(rgb_dir, exist_ok=True)
        os.makedirs(gray_dir, exist_ok=True)
        cam.PushFilter(sens.ChFilterSave(rgb_dir))

    # Convert image to grayscale
    cam.PushFilter(sens.ChFilterGrayscale())

    # Visualize the grayscale image
    if vis:
        cam.PushFilter(
            sens.ChFilterVisualize(
                int(image_width / 2),
                int(image_height / 2),
                "Grayscale Image",
            )
        )

    # Save grayscale images
    if save:
        cam.PushFilter(sens.ChFilterSave(gray_dir))

    # Resize grayscale image
    cam.PushFilter(
        sens.ChFilterImageResize(
            int(image_width / 2),
            int(image_height / 2),
        )
    )

    # Provide host access to the resized grayscale R8 buffer
    cam.PushFilter(sens.ChFilterR8Access())

    # Add the camera sensor to the manager
    manager.AddSensor(cam)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 7.0
    orbit_height = 3.0
    orbit_rate = 0.5
    ch_time = 0.0

    t1 = time.time()

    while ch_time < end_time:
        theta = ch_time * orbit_rate

        # Dynamically orbit the camera around the box.
        # At t=0 this matches the requested offset pose position: (-7, 0, 3).
        cam.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(theta),
                    -orbit_radius * math.sin(theta),
                    orbit_height,
                ),
                chrono.QuatFromAngleAxis(theta, chrono.ChVector3d(0, 0, 1)),
            )
        )

        # Update sensor manager: render/save/filter data
        manager.Update()

        # Access the RGBA8 buffer from the camera
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print(
                "RGBA8 buffer received from cam. Camera resolution: {0}x{1}".format(
                    rgba8_buffer.Width,
                    rgba8_buffer.Height,
                )
            )
            print("First Pixel: {0}".format(rgba8_data[0, 0, :]))

        # Perform one dynamics step
        mphysicalSystem.DoStepDynamics(step_size)

        # Update simulation time
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


# -----------------
# Object parameters
# -----------------
side = 2.0

# -----------------
# Camera parameters
# -----------------

# Noise model attached to the sensor
noise_model = "CONST_NORMAL"

# Update rate in Hz
update_rate = 30

# Image width and height
image_width = 1280
image_height = 720

# Camera horizontal field of view in radians
fov = 1.408

# Lag in seconds between sensing and data accessibility
lag = 0

# Exposure time in seconds
exposure_time = 0

# ---------------------
# Simulation parameters
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


if __name__ == "__main__":
    main()