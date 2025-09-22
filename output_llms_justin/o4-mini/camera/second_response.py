import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------------------
    # Add a mesh to be sensed by a camera
    # -----------------------------------
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(
        chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"),
        mergeenvelopes=False,
        displacenormals=True
    )
    # Scale the mesh uniformly by 2
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # Only one point light remains
    intensity = 1.0
    manager.scene.AddPointLight(
        chrono.ChVector3f(2, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0
    )
    # You can still keep an area light if you wish:
    manager.scene.AddAreaLight(
        chrono.ChVector3f(0, 0, 4),
        chrono.ChColor(intensity, intensity, intensity),
        500.0,
        chrono.ChVector3f(1, 0, 0),
        chrono.ChVector3f(0, -1, 0)
    )

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    # Changed offset to (-7, 0, 2)
    offset_pose = chrono.ChFrameD(
        chrono.ChVector3d(-7, 0, 2),
        chrono.QuatFromAngleAxis(2.0, chrono.ChVector3d(0, 1, 0))
    )

    cam = sens.ChCameraSensor(
        mesh_body,              # attach to this body
        update_rate,            # Hz
        offset_pose,            # pose of camera in body frame
        image_width,            # px
        image_height,           # px
        fov                     # rad, horizontal FOV
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
    # NONE => no noise

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale"))

    cam.PushFilter(sens.ChFilterRGBA8Access())

    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    cam.PushFilter(sens.ChFilterGrayscale())

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width // 2, image_height // 2, "Grayscale"))

    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    cam.PushFilter(sens.ChFilterImageResize(image_width // 2, image_height // 2))
    cam.PushFilter(sens.ChFilterR8Access())

    manager.AddSensor(cam)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10.0
    orbit_rate   = 0.1    # changed from 0.5 to 0.1
    ch_time      = 0.0

    t_wall_start = time.time()

    while ch_time < end_time:
        # Move the camera around in a horizontal circle
        cam.SetOffsetPose(chrono.ChFrameD(
            chrono.ChVector3d(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1.0
            ),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        ))

        # Try to grab the latest RGBA8 buffer
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print(f"Got RGBA8 frame ({rgba8_buffer.Width}×{rgba8_buffer.Height})")
            print("First pixel RGBA:", rgba8_data[0, 0, :])

        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t_wall_start)


# -----------------
# Camera parameters
# -----------------

noise_model   = "CONST_NORMAL"  # {"CONST_NORMAL","PIXEL_DEPENDENT","NONE"}
# lens_model    = sens.PINHOLE    # <— removed, not used
update_rate   = 30               # Hz
image_width   = 960              # px (changed from 1280)
image_height  = 480              # px (changed from 720)
fov           = 1.408            # rad
lag           = 0.0              # sec
exposure_time = 0.0              # sec

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time  = 20.0

# enable saving of images
save = True                      # changed from False
vis  = True

out_dir = "SENSOR_OUTPUT/"

if __name__ == "__main__":
    main()