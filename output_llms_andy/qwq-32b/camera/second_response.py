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
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    
    # Correctly scale the mesh using a scaling matrix
    scale_matrix = chrono.ChMatrix33d()
    scale_matrix.SetScale(2)
    mmesh.Transform(chrono.ChVectorD(0, 0, 0), scale_matrix)

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    intensity = 1.0
    # Remove extra point lights
    manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVectorF(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVectorF(1, 0, 0), chrono.ChVectorF(0, -1, 0))

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    # Corrected offset pose with ChFrameD and proper quaternion creation
    quat = chrono.ChQuaternionD()
    quat.Q_from_AngAxis(2, chrono.ChVectorD(0, 1, 0))
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-7, 0, 2), quat)

    cam = sens.ChCameraSensor(
        mesh_body,
        update_rate,
        offset_pose,
        image_width,
        image_height,
        fov
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    # Filters and other configurations remain unchanged except for the parameters

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
        cam.PushFilter(sens.ChFilterVisualize(int(image_width/2), int(image_height/2), "Grayscale Image"))
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))
    cam.PushFilter(sens.ChFilterImageResize(int(image_width/2), int(image_height/2)))
    cam.PushFilter(sens.ChFilterR8Access())

    manager.AddSensor(cam)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        current_angle = ch_time * orbit_rate
        quat = chrono.ChQuaternionD()
        quat.Q_from_AngAxis(current_angle, chrono.ChVectorD(0, 0, 1))
        position = chrono.ChVectorD(
            -orbit_radius * math.cos(current_angle),
            -orbit_radius * math.sin(current_angle),
            1
        )
        frame = chrono.ChFrameD(position, quat)
        cam.SetOffsetPose(frame)

        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print(f'RGBA8 buffer received from cam. Resolution: {rgba8_buffer.Width}x{rgba8_buffer.Height}')
            print(f'First Pixel: {rgba8_data[0, 0, :]}')

        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Camera parameters
# -----------------
noise_model = "CONST_NORMAL"
update_rate = 30
image_width = 960
image_height = 480
fov = 1.408
lag = 0
exposure_time = 0

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 20.0
save = True
vis = True
out_dir = "SENSOR_OUTPUT/"

main()