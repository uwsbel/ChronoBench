"""Camera-sensor scene: an orbiting RGB camera images a fixed triangular mesh.

Model
-----
A single triangular mesh (loaded from a Wavefront .obj) is added to the scene as a
FIXED rigid body. An OptiX-backed `ChCameraSensor`, driven by a `ChSensorManager`,
is attached to that body; a Gaussian noise filter plus visualize/save/RGBA8-access
filters post-process every frame. The camera's offset pose is updated each step so it
ORBITS the mesh on a circle of fixed radius, and the most-recent RGBA8 buffer (camera
resolution + first pixel) is printed at every step.

System type
-----------
`ChSystemNSC` (non-smooth contact). The mesh body carries a triangle-mesh collision
shape so the OptiX sensor can render it (the sensor only images bodies with collision
geometry); a Bullet collision system is selected accordingly. A standard Irrlicht
window provides an interactive third-person view of the same scene.

Expected behavior
-----------------
The mesh stays put (fixed body, no dynamics to integrate beyond a trivial step), while
the sensor camera sweeps a full circle around it; the saved sensor frames therefore
show the mesh from continuously changing azimuths, and the RGBA8 buffer prints a steady
1280x720 resolution once the sensor has produced its first image.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants === geometry / camera / simulation parameters
TIME_STEP = 1e-3                  # dynamics step (s)
SIM_END = 20.0                    # total simulated time (s)
UPDATE_RATE = 30.0                # camera update rate (Hz)
IMG_W, IMG_H = 1280, 720          # camera image resolution
CAM_FOV = 1.408                   # horizontal field of view (rad)
ORBIT_RADIUS = 10.0               # camera orbit radius about the mesh (m)
ORBIT_RATE = 0.5                  # orbit angular rate (rad/s)
CAM_HEIGHT = 1.0                  # camera height above mesh origin (m)
MESH_SCALE = 2.0                  # uniform scale applied to the loaded mesh
MESH_FILE = "vehicle/hmmwv/hmmwv_chassis.obj"   # Wavefront triangular mesh
NOISE_MEAN, NOISE_STDEV = 0.0, 0.02             # Gaussian camera-noise parameters

RENDER_FPS = 50.0                 # Irrlicht review-render cadence (frames/s)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once: physics steps per frame


# === System & gravity === NSC system with a Bullet collision system for the imaged mesh
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required: mesh has collision geometry the sensor images
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Bodies === fixed triangular mesh loaded from a Wavefront .obj
contact_mat = chrono.ChContactMaterialNSC()   # contact material for the mesh collision shape
contact_mat.SetFriction(0.8)
contact_mat.SetRestitution(0.0)

mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile(MESH_FILE), False, True)
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(MESH_SCALE))   # uniform scale

trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("scene_mesh")
trimesh_shape.SetMutable(False)

mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.SetFixed(True)
mesh_body.AddVisualShape(trimesh_shape)
# Collision geometry so the OptiX sensor renders the mesh (visual shapes alone are invisible to it).
coll_shape = chrono.ChCollisionShapeTriangleMesh(contact_mat, mmesh, True, True, 0.005)
mesh_body.AddCollisionShape(coll_shape)
mesh_body.EnableCollision(True)
sys.Add(mesh_body)

mesh_center = mesh_body.GetPos()   # cache: mesh origin, reused as the camera look-at target

# === Sensor manager === oversees the camera sensor + provides scene lighting for it
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# === Camera sensor === RGB camera attached to the mesh body, post-processed by a filter graph
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-ORBIT_RADIUS, 0, CAM_HEIGHT),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 0, 1)),
)
cam = sens.ChCameraSensor(mesh_body, UPDATE_RATE, offset_pose, IMG_W, IMG_H, CAM_FOV)
cam.SetName("orbit_camera")
cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(NOISE_MEAN, NOISE_STDEV))  # Gaussian image noise
cam.PushFilter(sens.ChFilterVisualize(IMG_W, IMG_H, "Camera Sensor"))         # live preview window
cam.PushFilter(sens.ChFilterSave("cam/sensor_rgb/"))                          # PNG frames -> sensor mp4
cam.PushFilter(sens.ChFilterRGBA8Access())                                    # host access to RGBA8 buffer
manager.AddSensor(cam)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(IMG_W, IMG_H)
vis.SetWindowTitle("Orbiting camera sensor over a triangular mesh")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-ORBIT_RADIUS, -ORBIT_RADIUS, 6), mesh_center)
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 20, 20, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === orbit the camera around the mesh, pump sensors, print buffer data

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            angle = t * ORBIT_RATE
            cam_x = -ORBIT_RADIUS * math.cos(angle)
            cam_y = -ORBIT_RADIUS * math.sin(angle)
            cam.SetOffsetPose(chrono.ChFramed(
                chrono.ChVector3d(cam_x, cam_y, CAM_HEIGHT),
                chrono.QuatFromAngleAxis(angle, chrono.ChVector3d(0, 0, 1))))

            # Print the most-recent camera buffer data (guard: empty until first sensor tick).
            rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
            if rgba8_buffer.HasData():
                rgba8_data = rgba8_buffer.GetRGBA8Data()
                print("RGBA8 buffer received. Camera resolution: {0}x{1}".format(
                    rgba8_buffer.Width, rgba8_buffer.Height))
                print("First pixel: {0}".format(rgba8_data[0, 0, :]))

            manager.Update()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
