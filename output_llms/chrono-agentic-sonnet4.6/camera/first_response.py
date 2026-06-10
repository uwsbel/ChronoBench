"""
Triangular mesh visualization with orbiting camera sensor.

A Wavefront .obj mesh is loaded and visualized as a fixed body in the scene.
A ChSensorManager is created and a ChCameraSensor with noise filters is attached
to the mesh body. During simulation, the camera's offset pose is updated at each
step to orbit around the mesh, and camera buffer data (RGBA8) is printed.

System: ChSystemNSC, Z-up, no gravity (fixed body).
Bodies: one fixed mesh body loaded from shoe_view.obj.
Sensors: one orbiting ChCameraSensor with noise, visualize, and save filters.
Expected: camera orbits the mesh continuously and sensor buffer data is printed.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Simulation constants ===
TIME_STEP    = 1e-3          # physics step size (s)
SIM_END      = 10.0          # total simulation time (s)
RENDER_FPS   = 50.0          # Irrlicht render cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Camera orbit parameters
ORBIT_RADIUS = 0.8           # orbit radius around the mesh (m)
ORBIT_HEIGHT = 0.3           # orbit height above origin (m)
ORBIT_SPEED  = 1.0           # angular speed (rad/s)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))   # fixed body, gravity off
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for collision shapes

# === Bodies — fixed triangular mesh loaded from .obj ===
mesh_path = chrono.GetChronoDataFile("models/bulldozer/shoe_view.obj")
trimesh = chrono.ChTriangleMeshConnected()
trimesh.LoadWavefrontMesh(mesh_path, True, True)
# No scaling — the mesh is already ~0.3m, appropriate for the sensor camera scene

mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))

# Visual shape for Irrlicht
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(trimesh)
vis_shape.SetMutable(False)
mesh_body.AddVisualShape(vis_shape)

# Collision shape so OptiX sensor can see the mesh
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.4)
col_shape = chrono.ChCollisionShapeTriangleMesh(mat, trimesh, False, False, 0.001)
mesh_body.AddCollisionShape(col_shape)
mesh_body.EnableCollision(True)

sys.AddBody(mesh_body)

# === Sensor manager — orbiting camera sensor ===
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0.5, 0.5, 2.0),
    chrono.ChColor(intensity, intensity, intensity),
    50.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-1.0, -1.0, 1.0),
    chrono.ChColor(intensity, intensity, intensity),
    30.0,
)

# Initial camera offset pose — orbit starts at angle 0
initial_angle = 0.0
cx0 = ORBIT_RADIUS * math.cos(initial_angle)
cy0 = ORBIT_RADIUS * math.sin(initial_angle)
cz0 = ORBIT_HEIGHT
yaw0 = math.atan2(-cy0, -cx0)                                   # aim toward origin
look_quat0 = chrono.QuatFromAngleAxis(yaw0, chrono.ChVector3d(0, 0, 1))

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(cx0, cy0, cz0),
    look_quat0,
)

cam = sens.ChCameraSensor(
    mesh_body,             # attached to the fixed mesh body
    30,                    # update_rate Hz — physical rate
    offset_pose,
    1280, 720,
    1.408,                 # horizontal FOV (rad)
)
cam.SetName("Orbiting Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Noise filter (constant-normal noise as requested)
cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
# Live preview window (visualize the sensor output)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera Orbit"))
# Host access to read buffer data each step
cam.PushFilter(sens.ChFilterRGBA8Access())
# Save PNG frames for per-sensor video
cam.PushFilter(sens.ChFilterSave("cam/orbit_cam/"))

manager.AddSensor(cam)

# === Visualization — Irrlicht window ===
# Initial Irrlicht camera at the same initial orbit position for review video clarity
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Triangular Mesh - Orbiting Camera Sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(cx0, cy0, cz0), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


# === Main loop ===
# Orbit angle increments each physics step; sensor and Irrlicht cameras both orbit.
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        # Current orbit angle and camera position (precomputed for this frame)
        t_now = sys.GetChTime()
        angle_now = ORBIT_SPEED * t_now
        cx_now = ORBIT_RADIUS * math.cos(angle_now)
        cy_now = ORBIT_RADIUS * math.sin(angle_now)

        # Update Irrlicht window camera to follow the orbit (review video shows motion)
        vis.UpdateCamera(
            chrono.ChVector3d(cx_now, cy_now, ORBIT_HEIGHT),
            chrono.ChVector3d(0, 0, 0),
        )

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            angle = ORBIT_SPEED * t
            cx = ORBIT_RADIUS * math.cos(angle)
            cy = ORBIT_RADIUS * math.sin(angle)
            cz = ORBIT_HEIGHT
            yaw = math.atan2(-cy, -cx)              # face the origin
            q = chrono.QuatFromAngleAxis(yaw, chrono.ChVector3d(0, 0, 1))
            cam.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(cx, cy, cz), q))

            manager.Update()                        # update sensor every physics step

            # Print buffer data each step (guarded: buffer empty until first sensor tick)
            buf = cam.GetMostRecentRGBA8Buffer()    # may be empty before first sensor tick
            if buf.HasData():                       # guard: skip frames sensor hasn't filled
                rgba = buf.GetRGBA8Data()
                print(f"t={t:.4f}s  cam=({cx:.3f},{cy:.3f},{cz:.3f})  "
                      f"buf_shape=({buf.Width}x{buf.Height})  "
                      f"pixel[0]={list(rgba[0, 0, :])}")


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:          # solver divergence / bad physics state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
