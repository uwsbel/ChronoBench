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

    # Add point lights (only one remains as requested)
    intensity = 1.0
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    
    # Fixed area light with correct parameters
    manager.scene.AddAreaLight(
        chrono.ChVector3f(0, 0, 4), 
        chrono.ChColor(intensity, intensity, intensity), 
        500.0,  # Intensity
        1.0,    # Width (added)
        1.0,    # Height (added)
        chrono.ChVector3f(1, 0, 0), 
        chrono.ChVector3f(0, -1, 0)
    )

    # ------------------------------------------------
    # Create a camera with modified parameters
    # ------------------------------------------------
    # Changed offset position: (-7, 0, 2) instead of (-5, 0, 2)
    # Corrected frame class: ChFrameD instead of ChFramed
    offset_pose = chrono.ChFrameD(
        chrono.ChVector3d(-7, 0, 2), 
        chrono.QuatFromAngleAxis(math.radians(2), chrono.ChVector3d(0, 1, 0))  # Corrected angle conversion
    )

    # Updated image dimensions: 960x480 instead of 1280x720
    cam = sens.ChCameraSensor(
        mesh_body,
        update_rate,
        offset_pose,
        960,    # Modified width
        480,    # Modified height
        fov
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    # ------------------------------------------------------------------
    # Filter graph
    # ------------------------------------------------------------------
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(960, 480, "Before Grayscale Filter"))  # Updated dimensions

    cam.PushFilter(sens.ChFilterRGBA8Access())

    # Enabled saving as requested
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    cam.PushFilter(sens.ChFilterGrayscale())

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(int(960/2), int(480/2), "Grayscale Image"))  # Updated dimensions

    # Enabled saving as requested
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    cam.PushFilter(sens.ChFilterImageResize(int(960/2), int(480/2)))  # Updated dimensions
    cam.PushFilter(sens.ChFilterR8Access())
    manager.AddSensor(cam)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1  # Changed from 0.5 to 0.1 rad/s
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        # Corrected frame class: ChFrameD instead of ChFramed
        cam.SetOffsetPose(chrono.ChFrameD(
            chrono.ChVector3d(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1
            ),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        ))

        # Access camera buffer
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer and rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print(f'RGBA8 buffer received. Resolution: {rgba8_buffer.Width}x{rgba8_buffer.Height}')
            print(f'First Pixel: {rgba8_data[0, 0, :]}')

        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print(f"Sim time: {end_time}, Wall time: {time.time() - t1}")

# -----------------
# Camera parameters
# -----------------
noise_model = "CONST_NORMAL"
update_rate = 30
image_width = 960   # Updated width
image_height = 480  # Updated height
fov = 1.408
lag = 0
exposure_time = 0

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 20.0
save = True  # Enabled saving as requested
vis = True
out_dir = "SENSOR_OUTPUT/"

# Removed unused lens_model as requested

# Execute
main()