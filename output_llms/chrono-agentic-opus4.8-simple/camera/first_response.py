import os
import math
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC physical system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # collision system (mesh body)

# --- load the triangular mesh from a Wavefront .obj and attach it to a fixed body ---
mesh = chrono.ChTriangleMeshConnected()                               # empty triangle mesh
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/geometries/suzanne.obj"), False, True)  # load .obj
mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1.0))   # no scaling / offset

mesh_body = chrono.ChBody()                                           # body that carries the mesh
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))                          # at the world origin
mesh_body.SetFixed(True)                                              # fixed (does not move)

trimesh_shape = chrono.ChVisualShapeTriangleMesh()                    # visual triangle-mesh shape
trimesh_shape.SetMesh(mesh)                                           # bind the loaded mesh
trimesh_shape.SetName("suzanne")                                      # asset name
trimesh_shape.SetMutable(False)                                       # static geometry
mesh_body.AddVisualShape(trimesh_shape, chrono.ChFramed())            # attach at the body frame
sys.Add(mesh_body)                                                    # add to the system

# --- sensor manager + scene lighting (camera needs lights) ---
manager = sens.ChSensorManager(sys)                                   # oversees all sensors
intensity = 1.0                                                       # light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# --- camera sensor attached to the mesh body, orbiting via SetOffsetPose each step ---
update_rate = 30.0                                                    # camera physical rate (Hz)
image_width = 1280                                                    # image width (px)
image_height = 720                                                    # image height (px)
fov = 1.408                                                           # horizontal FOV (rad)
orbit_radius = 5.0                                                    # orbit radius about the mesh
orbit_rate = 0.25                                                     # orbit angular rate (rad/s)
offset_pose = chrono.ChFramed(                                        # initial offset pose on the body
    chrono.ChVector3d(-orbit_radius, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(mesh_body, update_rate, offset_pose, image_width, image_height, fov)  # build camera
cam.SetName("Camera Sensor")                                         # sensor name
cam.SetLag(0)                                                        # no lag
cam.SetCollectionWindow(0)                                           # instantaneous exposure

# --- filter chain (order matters); noise + visualizations on the camera images ---
cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))      # constant-normal image noise
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "RGB Camera"))  # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                          # host access to the RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                       # save color PNG frames
manager.AddSensor(cam)                                              # register the camera

# --- Irrlicht visualization window (Initialize first, then scene elements; NO grid) ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Sensor on Mesh")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

time_step = 1e-3                                                     # integration step
sim_end = 12.0                                                       # end time
render_fps = 50.0                                                    # render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        t = sys.GetChTime()                                         # current sim time
        theta = orbit_rate * t                                      # orbit angle about the mesh
        cam_pos = chrono.ChVector3d(orbit_radius * math.cos(theta), orbit_radius * math.sin(theta), 1)  # orbit position
        cam_rot = chrono.QuatFromAngleAxis(theta + chrono.CH_PI, chrono.ChVector3d(0, 0, 1))  # aim +X back at the mesh
        cam.SetOffsetPose(chrono.ChFramed(cam_pos, cam_rot))       # update the camera pose (orbit)
        manager.Update()                                           # pump sensors once per step
        sys.DoStepDynamics(time_step)                             # advance physics
        buffer = cam.GetMostRecentRGBA8Buffer()                   # latest camera buffer
        if buffer.HasData():                                      # only when the sensor has produced data
            print('Camera buffer received. Resolution: {0}x{1}'.format(buffer.Width, buffer.Height))
        if sys.GetChTime() >= sim_end:
            break
