import pychrono as chrono
import pychrono.sensor as sens

import time
import math


def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------------------
    # add a mesh to be sensed, as a body
    # -----------------------------------
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile(
        "vehicle/mmarm/mmesh.obj"), False, True)
    # scale to a different size
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("Sensable object TriMesh shape")
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
    # offset of the sensor in the chasis cs
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-8, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    intensity = 1.0

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    # ------------------------
    # Camera's optical settings
    # ------------------------
    image_width = 1280
    image_height = 720
    title = "Camera Sensor"

    # ---------------
    # Camera's noise models
    # ---------------
    lens_focal_length = 0
    noise_model = sens.NoiseModel_NONE

    # ------------------
    # Camera's collection
    # ------------------
    exposure_time = 0  # [s]
    lag = 0            # [s]

    # ---------------------
    # Filter for lens distortion
    # ---------------------
    lens_distortion = sens.LensDistortionNONE

    # ------------------
    # Renderers for camera
    # ------------------
    window_width = 1280
    window_height = 720
    granularity = 2
    cam = sens.ChCameraSensor(
        mesh_body,              # body camera is attached to
        image_width,            # image width
        image_height,           # image height
        title,                  # window title
        offset_pose,            # offset pose
        exposure_time,          # exposure time (determines updating rate)
        lag,                    # rendering lag
    )
    cam.SetName("Camera Sensor")

    if (lens_focal_length > 0):
        cam.SetLensFocalLength(lens_focal_length)
    if (noise_model == sens.NoiseModel_GAUSSIAN):
        cam.SetNoiseModel(sens.GaussianNoiseModel())
    elif (noise_model == sens.NoiseModel_UNIFORM):
        cam.SetNoiseModel(sens.UniformNoiseModel())

    if (lens_distortion == sens.LensDistortionRADTAN):
        cam.SetLensedRadialTangentDistortion(0.5, 0.5)
    elif (lens_distortion == sens.LensDistortionFOV):
        cam.SetLensedFovDistortion(0.5, 0.5)

    # Render the image at half the camera's resolution
    cam.PushFilter(sens.ChFilterVisualize(
        window_width, window_height, granularity))

    # Access the camera's filter graph
    graph = cam.GetVacuumFilterGraph()
    # Render the image at the camera's resolution
    graph.PushFilter(sens.ChFilterVisualize(
        image_width, image_height, 1))

    # Add the camera to the sensor manager
    manager.AddSensor(cam)

    # ---------------
    # Create a filter graph for the camera
    # ---------------
    # Render the image at half the camera's resolution
    cam.PushFilter(sens.ChFilterVisualize(
        window_width, window_height, granularity))

    # Access the camera's filter graph
    graph = cam.GetVacuumFilterGraph()
    # Render the image at the camera's resolution
    graph.PushFilter(sens.ChFilterVisualize(
        image_width, image_height, 1))

    # ---------------
    #  Create a filter graph for the camera's depth buffer
    # ---------------
    # Render the depth buffer at the camera's resolution
    graph_depth = cam.GetDepthFilterGraph()
    graph_depth.PushFilter(sens.ChFilterVisualize(
        image_width, image_height, 1))

    # ------------------
    # Just to check the output
    # ------------------
    # Access the camera's filter graph
    graph = cam.GetVacuumFilterGraph()
    # Save the RGBA8 buffer to a png file at the end of a simulation
    # graph.PushFilter(sens.ChFilterSave(out_dir+"camera/"))

    # --------------
    # Initialize manager
    # --------------
    manager.Initialize()

    # ---------------
    # Simulation loop
    # ---------------
    orbit_radius = 8
    orbit_rate = 0.5
    cam_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-orbit_radius, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    t = 0
    orbit_angle = 0
    render_step_size = 1.0 / 60  # FPS
    render_step_number = 0
    ch_time = 0

    while (ch_time < 10):

        small_step = orbit_rate * render_step_size
        orbit_angle += small_step
        cam_offset_pose = chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(orbit_angle), -orbit_radius * math.sin(orbit_angle), 1), chrono.QuatFromAngleAxis(orbit_angle, chrono.ChVector3d(0, 0, 1)))
        cam.SetOffsetPose(cam_offset_pose)

        # Access the camera's RGBA8 data from the filter graph
        rgba8_buffer = graph.GetMostRecentBuffer().GetRGBA8Buffer()
        depth16_buffer = graph_depth.GetMostRecentBuffer().GetDepth16Buffer()

        # Update sensor manager
        # Will render/save/filter automatically
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(render_step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

        print("buffer width: ", rgba8_buffer.Width)
        print("buffer height: ", rgba8_buffer.Height)
        print("buffer data: ", rgba8_buffer.Data)
        print("\n")

    return 0


if __name__ == "__main__":
    main()