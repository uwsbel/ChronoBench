import pychrono.core as chrono
import pychrono.sensor as sens

import math
import os
import time


def main():
    
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)

    
    box_body.GetVisualShape(0).SetTexture(
        chrono.GetChronoDataFile("textures/checker2.png")
    )

    
    mphysicalSystem.Add(box_body)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
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

    
    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-7, 0, 3),
        chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)),
    )

    
    cam = sens.ChCameraSensor(
        box_body,        
        update_rate,     
        offset_pose,     
        image_width,     
        image_height,    
        fov,             
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
    else:
        raise ValueError(f"Unsupported noise model: {noise_model}")

    
    if vis:
        cam.PushFilter(
            sens.ChFilterVisualize(
                image_width,
                image_height,
                "Before Grayscale Filter",
            )
        )

    
    cam.PushFilter(sens.ChFilterRGBA8Access())

    
    if save:
        rgb_dir = os.path.join(out_dir, "rgb") + os.sep
        gray_dir = os.path.join(out_dir, "gray") + os.sep
        os.makedirs(rgb_dir, exist_ok=True)
        os.makedirs(gray_dir, exist_ok=True)
        cam.PushFilter(sens.ChFilterSave(rgb_dir))

    
    cam.PushFilter(sens.ChFilterGrayscale())

    
    if vis:
        cam.PushFilter(
            sens.ChFilterVisualize(
                int(image_width / 2),
                int(image_height / 2),
                "Grayscale Image",
            )
        )

    
    if save:
        cam.PushFilter(sens.ChFilterSave(gray_dir))

    
    cam.PushFilter(
        sens.ChFilterImageResize(
            int(image_width / 2),
            int(image_height / 2),
        )
    )

    
    cam.PushFilter(sens.ChFilterR8Access())

    
    manager.AddSensor(cam)

    
    
    
    orbit_radius = 7.0
    orbit_height = 3.0
    orbit_rate = 0.5
    ch_time = 0.0

    t1 = time.time()

    while ch_time < end_time:
        theta = ch_time * orbit_rate

        
        
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

        
        manager.Update()

        
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

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)





side = 2.0






noise_model = "CONST_NORMAL"


update_rate = 30


image_width = 1280
image_height = 720


fov = 1.408


lag = 0


exposure_time = 0






step_size = 1e-3


end_time = 20.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"


if __name__ == "__main__":
    main()