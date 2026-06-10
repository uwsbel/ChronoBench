"""Camera sensor demo — orbiting RGB camera around a fixed mesh.

Models a single fixed triangle-mesh body (a scaled HMMWV chassis) sensed by an
OptiX RGB ``ChCameraSensor`` that orbits the mesh. System type: NSC (no contact —
the mesh is fixed and nothing else moves, so no collision system is needed). The
camera rides on the mesh body and its offset pose is rewritten every step to sweep
a circular orbit; its filter graph saves both the color and the grayscale image
streams. Expected behavior: the saved frames show the chassis from a viewpoint that
revolves slowly around it.
"""

import math
import pychrono.core as chrono
import pychrono.sensor as sens

# === Parameters === camera + simulation constants (final desired values)
update_rate = 30                 # camera update rate (Hz) — physical rate, not 1/dt
image_width = 960                # image width (px)
image_height = 480               # image height (px)
fov = 1.408                      # horizontal field of view (rad)
lag = 0                          # lag between sensing and data availability (s)
exposure_time = 0                # collection / exposure window (s)
step_size = 1e-3                 # integration step size (s)
end_time = 20.0                  # simulation end time (s)
orbit_radius = 10                # camera orbit radius about the mesh (m)
orbit_rate = 0.1                 # camera orbit angular rate (rad/s)
out_dir = "cam/"                 # camera image output directory
save = True                      # save camera images to PNG
do_vis = True                    # render live preview windows


# === System & gravity === NSC rigid-body system; no contact in this scene
mphysicalSystem = chrono.ChSystemNSC()

# === Bodies === one fixed triangle-mesh body to be sensed by the camera
mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))     # scale up the mesh

trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("HMMWV Chassis Mesh")
trimesh_shape.SetMutable(False)

mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.AddVisualShape(trimesh_shape)
mesh_body.SetFixed(True)
mphysicalSystem.Add(mesh_body)

# === Sensor manager & lighting === point + area lights illuminate the camera scene
manager = sens.ChSensorManager(mphysicalSystem)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0,
                           chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

# === Camera sensor === orbiting RGB camera attached to the mesh body
offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 2),
                              chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(mesh_body, update_rate, offset_pose, image_width, image_height, fov)
cam.SetName("Camera Sensor")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)

# === Filter graph === order matters; each ChFilterSave snapshots the buffer at its position
if do_vis:
    cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))
cam.PushFilter(sens.ChFilterRGBA8Access())                # host access to the RGBA8 buffer
if save:
    cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))   # save stream #1: color PNGs
cam.PushFilter(sens.ChFilterGrayscale())                  # convert to grayscale
if do_vis:
    cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))
if save:
    cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))  # save stream #2: grayscale PNGs
cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))
cam.PushFilter(sens.ChFilterR8Access())                   # host access to the resized R8 buffer
manager.AddSensor(cam)

# === Main loop === orbit the camera around the mesh; pump the sensor each step
ch_time = 0.0
try:
    while ch_time < end_time:
        # Rewrite the offset pose to sweep a circular orbit about the mesh.
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate),
                              -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()     # may be empty before the first tick
        if rgba8_buffer.HasData():                        # guard: skip frames not yet filled
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer received. Resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))

        manager.Update()                                  # render/save/filter all sensors
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()
except (RuntimeError, ValueError) as exc:                 # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble the saved sensor frames into review videos
