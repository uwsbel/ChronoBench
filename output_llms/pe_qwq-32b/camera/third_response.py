import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ---------------------
    # Create a box object
    # ---------------------
    side = 2.0  # Box dimensions (side length)
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)  # density 1000 kg/m^3
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))  # Position the box at origin
    box_body.SetFixed(True)  # Fix the box in space
    mphysicalSystem.Add(box_body)  # Add to the physical system

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
    # Define the camera offset pose relative to the box
    offset_pose = chrono.ChFramed(
        chrono.ChVectorD(-7, 0, 3),  # New position coordinates
        chrono.Q_from_AngAxis(2, chrono.ChVectorD(0, 1, 0))  # Rotation around Y-axis by 2 radians
    )

    # Initialize the camera sensor attached to the box
    cam = sens.ChCameraSensor(
        box_body,          # Attached to the box instead of mesh
        update_rate,       # Update rate in Hz
        offset_pose,       # New offset pose
        image_width,       # Image width
        image_height,      # Image height
        fov                # Field of view
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    # Noise filter setup
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))
    # "NONE" case does nothing

    # Visualization and data access filters
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale"))
    cam.PushFilter(sens.ChFilterRGBA8Access())
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    # Grayscale processing
    cam.PushFilter(sens.ChFilterGrayscale())
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(int(image_width/2), int(image_height/2), "Grayscale"))
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    # Resize and access filters
    cam.PushFilter(sens.ChFilterImageResize(int(image_width/2), int(image_height/2)))
    cam.PushFilter(sens.ChFilterR8Access())

    # Add to manager
    manager.AddSensor(cam)

    # ---------------
    # Simulation loop
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.5
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        # Update camera position dynamically
        angle = ch_time * orbit_rate
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVectorD(-orbit_radius * math.cos(angle), 
                            -orbit_radius * math.sin(angle), 
                            1),
            chrono.Q_from_AngAxis(angle, chrono.ChVectorD(0,0,1))
        ))

        # Access camera data
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            data = rgba8_buffer.GetRGBA8Data()
            print(f'RGBA8 buffer received. Resolution: {rgba8_buffer.Width}x{rgba8_buffer.Height}')
            print(f'First pixel: {data[0,0]}')

        # Update sensor and simulation
        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print(f"Simulation time: {end_time}, Wall time: {time.time()-t1}")

# ---------------------
# Simulation parameters
# ---------------------
side = 2.0  # Box dimensions

# Camera parameters
noise_model = "CONST_NORMAL"
update_rate = 30
image_width = 1280
image_height = 720
fov = 1.408  # Horizontal FOV in radians
lag = 0
exposure_time = 0

# Simulation parameters
step_size = 1e-3
end_time = 20.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"