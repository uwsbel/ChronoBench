import os
import math
import pychrono.core as chrono
import pychrono.sensor as sens

sys = chrono.ChSystemNSC()                                          # sensor demos use NSC
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # collision system

chrono.SetChronoDataPath("/home/hongyu/Documents/chrono-900/data/")  # Chrono data assets

mmesh = chrono.ChTriangleMeshConnected()                            # triangular mesh container
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)  # load .obj
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))  # uniform scale x2

trimesh_shape = chrono.ChVisualShapeTriangleMesh()                  # visual shape from the mesh
trimesh_shape.SetMesh(mmesh)                                        # bind the loaded mesh
trimesh_shape.SetName("HMMWV Chassis Mesh")                         # name the shape
trimesh_shape.SetMutable(False)                                     # static geometry

mesh_body = chrono.ChBody()                                         # body that carries the mesh
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))                        # at the origin
mesh_body.AddVisualShape(trimesh_shape)                             # attach the visual mesh
mesh_body.SetFixed(True)                                            # fixed in the scene
sys.Add(mesh_body)                                                  # add the body to the system

manager = sens.ChSensorManager(sys)                                 # manage all sensors
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)   # key light
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(1, 1, 1), 500.0,
                           chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))            # area light

offset_pose = chrono.ChFramed(                                      # camera offset pose on the body
    chrono.ChVector3d(-7, 0, 2),                                    # initial position offset
    chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0))         # orientation about Y
)

update_rate = 30                                                    # physical update rate, Hz
image_width = 960                                                   # camera image width, px
image_height = 480                                                  # camera image height, px
fov = 1.408                                                         # horizontal field of view, rad

cam = sens.ChCameraSensor(mesh_body, update_rate, offset_pose, image_width, image_height, fov)  # RGB camera
cam.SetName("Camera Sensor")                                        # name the sensor
cam.SetLag(0)                                                       # no signal lag
cam.SetCollectionWindow(0)                                          # instantaneous exposure

cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))      # constant-normal noise (mean, stdev)
cam.PushFilter(sens.ChFilterRGBA8Access())                         # host access to the RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                      # save color frames
cam.PushFilter(sens.ChFilterGrayscale())                          # convert image to grayscale
cam.PushFilter(sens.ChFilterSave("cam/gray/"))                    # save grayscale frames
cam.PushFilter(sens.ChFilterImageResize(int(image_width/2), int(image_height/2)))   # resize for downstream
cam.PushFilter(sens.ChFilterR8Access())                           # host access to the R8 buffer
manager.AddSensor(cam)                                             # register the camera

orbit_radius = 10                                                  # camera orbit radius, m
orbit_rate = 0.1                                                   # camera orbit rate, rad/s
step_size = 1e-3                                                   # physics step size, s
end_time = 20.0                                                    # simulation duration, s


time = 0                                                           # simulation clock
while time < end_time:
    time = sys.GetChTime()                                         # current simulation time
    cam.SetOffsetPose(chrono.ChFramed(                            # orbit the camera around the mesh
        chrono.ChVector3d(-orbit_radius * math.cos(time * orbit_rate),
                          -orbit_radius * math.sin(time * orbit_rate), 1),
        chrono.QuatFromAngleAxis(time * orbit_rate, chrono.ChVector3d(0, 0, 1))
    ))
    manager.Update()                                              # pump all sensors this step
    rgba8_buffer = cam.GetMostRecentRGBA8Buffer()                # latest camera buffer
    if rgba8_buffer.HasData():                                    # buffer filled by the sensor
        rgba8_data = rgba8_buffer.GetRGBA8Data()                 # numpy view of the image
        print('Camera buffer received. Resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
        print('First pixel RGBA: ', rgba8_data[0, 0, :])         # top-left pixel sample
    sys.DoStepDynamics(step_size)                                # advance the physics
