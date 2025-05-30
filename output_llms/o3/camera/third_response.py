import math
import os
import time

import pychrono.core as chrono
import pychrono.sensor as sens

# ------------
# User options
# ------------

# Noise model for the camera                                    (“CONST_NORMAL”, ”PIXEL_DEPENDENT”, ”NONE”)
noise_model   = "CONST_NORMAL"

# Camera parameters
update_rate   = 30.0                                # Hz
image_width   = 1280                                # pixels
image_height  = 720                                 # pixels
fov           = 1.408                               # rad
lag           = 0.0                                 # s
exposure_time = 0.0                                 # s

# Simulation parameters
step_size     = 1e-3                                # s
end_time      = 20.0                                # s

# Output/visualisation
save          = False                               # write PNGs
vis           = True                                # on–screen visualisation
out_dir       = "SENSOR_OUTPUT/"                    # image root folder

# Box size (edge length)
side          = 2.0                                 # m


def main():
    # ------------------------------------------------------------------------------
    # 1. Create the Chrono physical system
    # ------------------------------------------------------------------------------
    system = chrono.ChSystemNSC()

    # ------------------------------------------------------------------------------
    # 2. Add a simple fixed box that the camera will observe
    # ------------------------------------------------------------------------------
    box = chrono.ChBodyEasyBox(side, side, side,         # size
                               1000.0,                   # density (kg/m³)
                               True, True)               # visualisation / collision

    box.SetPos(chrono.ChVectorD(0.0, 0.0, side / 2.0))   # sit on the ground
    box.SetBodyFixed(True)

    # add a simple texture so the camera sees something
    tex_file = chrono.GetChronoDataFile("textures/white.png")
    for i in range(box.GetNumVisualShapes()):
        box.GetVisualShape(i).SetTexture(tex_file)

    system.Add(box)

    # ------------------------------------------------------------------------------
    # 3. Create a sensor-manager and some lights
    # ------------------------------------------------------------------------------
    manager = sens.ChSensorManager(system)

    light_int = 1.0
    manager.scene.AddPointLight(chrono.ChVectorF(2,   2.5, 4),
                                chrono.ChColor(light_int, light_int, light_int), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorF(-2,  2.5, 4),
                                chrono.ChColor(light_int, light_int, light_int), 500.0)
    manager.scene.AddAreaLight (chrono.ChVectorF(0,   0,   4),
                                chrono.ChColor(light_int, light_int, light_int), 500.0,
                                chrono.ChVectorF(1, 0, 0), chrono.ChVectorF(0, -1, 0))

    # ------------------------------------------------------------------------------
    # 4. Create a camera attached to the box
    # ------------------------------------------------------------------------------
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-7, 0, 3),
                                  chrono.Q_from_AngAxis(2.0, chrono.ChVectorD(0, 1, 0)))

    cam = sens.ChCameraSensor(box, update_rate, offset_pose,
                              image_width, image_height, fov)

    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    # optional noise
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))

    # filter graph ---------------------------------------------------------------
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height,
                                              "RGB Image (pre-gray)"))

    cam.PushFilter(sens.ChFilterRGBA8Access())
    if save:
        cam.PushFilter(sens.ChFilterSave(os.path.join(out_dir, "rgb/")))

    cam.PushFilter(sens.ChFilterGrayscale())

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width // 2, image_height // 2,
                                              "Grayscale Image"))
    if save:
        cam.PushFilter(sens.ChFilterSave(os.path.join(out_dir, "gray/")))

    cam.PushFilter(sens.ChFilterImageResize(image_width // 2, image_height // 2))
    cam.PushFilter(sens.ChFilterR8Access())

    manager.AddSensor(cam)

    # create output folders if required
    if save:
        os.makedirs(os.path.join(out_dir, "rgb/"),  exist_ok=True)
        os.makedirs(os.path.join(out_dir, "gray/"), exist_ok=True)

    # ------------------------------------------------------------------------------
    # 5. Simulation loop
    # ------------------------------------------------------------------------------
    orbit_radius = 10.0     # m
    orbit_rate   = 0.5      # rad/s

    ch_time   = 0.0
    wall_t0   = time.time()

    while ch_time < end_time:
        # continuously move the camera around the box
        cam.SetOffsetPose(
            chrono.ChFrameD(
                chrono.ChVectorD(-orbit_radius * math.cos(ch_time * orbit_rate),
                                 -orbit_radius * math.sin(ch_time * orbit_rate),
                                 3.0),
                chrono.Q_from_AngAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
            )
        )

        # access most recent image (for demonstration / debug)
        rgba_buf = cam.GetMostRecentRGBA8Buffer()
        if rgba_buf.HasData():
            data = rgba_buf.GetRGBA8Data()
            print(f"t = {ch_time:6.3f}  |  Image {rgba_buf.Width}x{rgba_buf.Height}  "
                  f"|  first pixel RGBA = {data[0, 0, :]}")

        manager.Update()                     # render + filter
        system.DoStepDynamics(step_size)     # advance physics
        ch_time = system.GetChTime()

    print(f"Simulation time {end_time} s  |  Wall time {time.time() - wall_t0:.2f} s")


# -------------
# main program
# -------------
if __name__ == "__main__":
    main()