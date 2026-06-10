"""
Camera Sensor Demo — PyChrono 9.0.x with Irrlicht + OptiX

Simulates a triangular mesh (Wavefront .obj) loaded as a fixed body in a
ChSystemNSC scene. A ChCameraSensor is attached to the mesh body, managed by a
ChSensorManager. Noise filters and visualization are applied to the camera images.
The camera orbits around the mesh by dynamically updating its offset pose each
step. Camera buffer data (RGBA8) is printed at each sensor update tick.

System:      ChSystemNSC, Z-up
Body:        fixed mesh body loaded from sensor/geometries/suzanne.obj
Sensor:      OptiX ChCameraSensor with ChFilterCameraNoiseConstNormal,
             ChFilterVisualize, ChFilterRGBA8Access, ChFilterSave
Renderer:    Irrlicht window (default) + OptiX sensor camera
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Simulation parameters ===
time_step: float = 1e-3       # physics step size (s)
sim_end: float = 10.0         # total simulation duration (s)
render_fps: float = 30.0      # Irrlicht render rate (Hz)
render_every: int = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Camera orbit parameters
CAM_ORBIT_RADIUS: float = 6.0    # orbit radius (m) around mesh centre
CAM_ORBIT_HEIGHT: float = 2.0    # camera height above XY plane (m)
CAM_ORBIT_SPEED: float = 0.5     # angular speed (rad/s) of orbit
CAM_UPDATE_RATE: int = 30        # OptiX camera physical update rate (Hz)
CAM_WIDTH: int = 1280
CAM_HEIGHT: int = 720
CAM_FOV: float = 1.408           # horizontal field-of-view (rad)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Collision system: required because the mesh body carries a collision shape
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies — load triangular mesh as a fixed body ===
# The mesh is a bundled Chrono sensor-demo asset; resolve via GetChronoDataFile.
mesh_path = chrono.GetChronoDataFile("sensor/geometries/suzanne.obj")

# Visual shape: render the OBJ mesh in Irrlicht
mesh_vis = chrono.ChTriangleMeshConnected()
mesh_vis.LoadWavefrontMesh(mesh_path, True, True)

vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(mesh_vis)
vis_shape.SetMutable(False)

# Collision shape: required so OptiX sensor can see the body
mesh_col = chrono.ChTriangleMeshConnected()
mesh_col.LoadWavefrontMesh(mesh_path, False, False)

mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.4)
mat.SetRestitution(0.0)

col_shape = chrono.ChCollisionShapeTriangleMesh(mat, mesh_col, False, False, 0.005)

# Build the fixed protagonist body
mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.AddVisualShape(vis_shape)
mesh_body.AddCollisionShape(col_shape)
mesh_body.EnableCollision(True)
sys.AddBody(mesh_body)

# === Sensor manager & scene lighting ===
manager = sens.ChSensorManager(sys)
intensity: float = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-5, -5, 10),
    chrono.ChColor(intensity, intensity, intensity),
    300.0,
)

# === Camera sensor — orbiting around the mesh ===
# Initial offset: camera at radius along +X axis, tilted toward origin.
initial_eye = chrono.ChVector3d(CAM_ORBIT_RADIUS, 0.0, CAM_ORBIT_HEIGHT)
initial_target = chrono.ChVector3d(0, 0, 0)

# Build offset pose — QuatFromAngleAxis aims local +X toward subject
fwd = (initial_target - initial_eye).GetNormalized()
yaw_init = math.atan2(fwd.y, fwd.x)
pitch_init = math.asin(-fwd.z)    # downward pitch toward mesh
look_quat = chrono.QuatFromAngleAxis(yaw_init, chrono.ChVector3d(0, 0, 1))

offset_pose = chrono.ChFramed(initial_eye, look_quat)

cam = sens.ChCameraSensor(
    mesh_body,           # attach to the fixed mesh body (its world frame = world)
    CAM_UPDATE_RATE,     # physical update_rate (Hz) — NOT 1/time_step
    offset_pose,
    CAM_WIDTH,
    CAM_HEIGHT,
    CAM_FOV,
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain: noise -> visualize -> access -> save
cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))          # Gaussian noise
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "RGB Camera"))  # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())                               # host buffer access
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                            # save color PNGs
cam.PushFilter(sens.ChFilterGrayscale())                                 # convert to gray
cam.PushFilter(sens.ChFilterVisualize(640, 360, "Grayscale Camera"))     # gray preview
cam.PushFilter(sens.ChFilterSave("cam/gray/"))                           # save gray PNGs
cam.PushFilter(sens.ChFilterImageResize(640, 360))
cam.PushFilter(sens.ChFilterR8Access())                                  # gray host access
manager.AddSensor(cam)

# === Irrlicht visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Sensor — Orbiting Mesh Demo")
vis.Initialize()                                              # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(8, -6, 4),
    chrono.ChVector3d(0, 0, 0),
)
vis.AddTypicalLights()
vis.AddGrid(
    1.0, 1.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1.5), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)


# === Main simulation loop ===
frame: int = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            t: float = sys.GetChTime()  # cache: current time, fetched once per physics step

            # === Dynamic camera orbit update ===
            # Recompute orbit angle and rebuild the sensor's offset pose each step
            angle: float = CAM_ORBIT_SPEED * t   # precomputed per step; orbit angle (rad)
            eye_x: float = CAM_ORBIT_RADIUS * math.cos(angle)
            eye_y: float = CAM_ORBIT_RADIUS * math.sin(angle)
            eye_z: float = CAM_ORBIT_HEIGHT
            eye_pos = chrono.ChVector3d(eye_x, eye_y, eye_z)

            fwd_vec = (chrono.ChVector3d(0, 0, 0) - eye_pos).GetNormalized()
            yaw: float = math.atan2(fwd_vec.y, fwd_vec.x)
            new_quat = chrono.QuatFromAngleAxis(yaw, chrono.ChVector3d(0, 0, 1))
            cam.SetOffsetPose(chrono.ChFramed(eye_pos, new_quat))

            # Update Irrlicht window camera to follow orbit for review
            vis.UpdateCamera(
                chrono.ChVector3d(eye_x * 1.5, eye_y * 1.5, eye_z + 2.0),
                chrono.ChVector3d(0, 0, 0),
            )

            manager.Update()  # pump all sensors exactly once per physics step

            # Print buffer data when available
            buf = cam.GetMostRecentRGBA8Buffer()   # cache: buffer handle per step
            buf_ready: bool = buf.HasData()         # guard: check once, reuse below
            if buf_ready:                           # skip frames before first sensor tick
                rgba = buf.GetRGBA8Data()           # safe after HasData()
                w = buf.Width
                h = buf.Height
                print(f"[t={t:.3f}s] Camera buffer: {w}x{h} pixels, "
                      f"sample RGBA=[{rgba[0]},{rgba[1]},{rgba[2]},{rgba[3]}]")

            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:      # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
