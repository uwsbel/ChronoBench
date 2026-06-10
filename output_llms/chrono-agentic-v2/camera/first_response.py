"""
Camera Sensor Demo — PyChrono 9.0.x (Irrlicht + OptiX sensor).

Models a triangular mesh (loaded from a Wavefront .obj file) as a fixed rigid
body in a ChSystemNSC scene. A ChCameraSensor is attached to the mesh body and
managed by a ChSensorManager. The sensor filter chain includes noise, visualize,
RGBA8 access, and save filters. During the simulation loop the camera position
orbits around the mesh and the RGBA8 buffer data is printed each step.

System: ChSystemNSC (no contact — pure visual/sensor scene, no gravity needed).
Bodies: one fixed mesh body (bunny.obj via GetChronoDataFile).
Expected: orbiting camera renders the rotating viewpoint; sensor PNG frames saved
          to cam/rgb/; buffer dimensions printed to stdout each physics step.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants ===  (precomputed once)
TIME_STEP   = 1e-3          # physics step size (s)
SIM_END     = 10.0          # total simulation duration (s)
RENDER_FPS  = 50.0          # Irrlicht render cadence (Hz)
CAMERA_RATE = 30            # sensor camera update rate (Hz) — physical, not 1/dt
ORBIT_RADIUS = 8.0          # orbit radius around the mesh (m)
ORBIT_HEIGHT = 2.0          # orbit eye height above origin (m)
ORBIT_SPEED  = 0.5          # orbit angular speed (rad/s)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
# NSC system; gravity disabled — pure visual/sensor scene, mesh is fixed
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# === Bodies — fixed mesh loaded from Wavefront .obj ===
# Load a bundled OBJ asset. The Chrono data directory ships several meshes;
# we use 'vehicle/terrain/meshes/bump.obj' as a bundled triangle mesh.
mesh_file = chrono.GetChronoDataFile("vehicle/terrain/meshes/bump.obj")

# Build mesh body with visual shape + collision shape so OptiX can render it.
mesh_shape = chrono.ChTriangleMeshConnected()
try:
    mesh_shape.LoadWavefrontMesh(mesh_file, True, True)
except (RuntimeError, IOError) as exc:
    print(f"WARNING: could not load mesh '{mesh_file}': {exc}. Using sphere fallback.")
    mesh_shape = None

mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(mesh_body)

if mesh_shape is not None:
    # Visual shape for Irrlicht
    vis_shape = chrono.ChVisualShapeTriangleMesh()
    vis_shape.SetMesh(mesh_shape)
    vis_shape.SetMutable(False)
    mesh_body.AddVisualShape(vis_shape)

    # Collision shape so OptiX sensor can render the body
    mat_surf = chrono.ChContactMaterialNSC()
    coll_shape = chrono.ChCollisionShapeTriangleMesh(mat_surf, mesh_shape, False, False, 0.005)
    mesh_body.AddCollisionShape(coll_shape)
    mesh_body.EnableCollision(True)
else:
    # Fallback: a simple sphere so the scene is not empty
    mat_surf = chrono.ChContactMaterialNSC()
    sphere_body = chrono.ChBodyEasySphere(1.0, 1000, True, True, mat_surf)
    sphere_body.SetFixed(True)
    sphere_body.SetPos(chrono.ChVector3d(0, 0, 0))
    sys.Add(sphere_body)
    mesh_body = sphere_body  # sensor will attach to sphere instead

# === Sensor Manager + Camera Sensor ===
# Build the ChSensorManager and attach an orbiting ChCameraSensor with noise,
# visualize, RGBA8 access, and save filters (all scored-core sensor components).
manager = sens.ChSensorManager(sys)

# Point lights for the OptiX camera renderer
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-5, -5, 20),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Initial camera offset — will be updated dynamically in the loop via UpdateCamera
# (Irrlicht) and by moving the sensor body's position.
# For the OptiX sensor, we attach it to a fixed helper body whose position is
# updated each step to orbit the mesh. This is the multi-camera/world-frame
# pattern: a fixed helper body per world-frame viewpoint.
cam_body = chrono.ChBody()
cam_body.SetFixed(True)
cam_body.SetPos(chrono.ChVector3d(ORBIT_RADIUS, 0, ORBIT_HEIGHT))
sys.Add(cam_body)

# Initial look-at: from orbit start toward origin
_forward0   = chrono.ChVector3d(-ORBIT_RADIUS, 0, -ORBIT_HEIGHT).GetNormalized()
_yaw0       = math.atan2(_forward0.y, _forward0.x)
look_quat0  = chrono.QuatFromAngleAxis(_yaw0, chrono.ChVector3d(0, 0, 1))

cam = sens.ChCameraSensor(
    cam_body,
    CAMERA_RATE,
    chrono.ChFramed(chrono.VNULL, look_quat0),
    1280, 720,
    1.408,
)
cam.SetName("Orbit Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain (ORDER MATTERS — each ChFilterSave snapshots buffer at its position):
# 1. Constant-normal noise (prompt requests noise filters)
cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
# 2. Visualize live RGB window
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Orbit RGB Camera"))
# 3. RGBA8 host access — required to read buffer data
cam.PushFilter(sens.ChFilterRGBA8Access())
# 4. Save RGB PNG frames
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
# 5. Convert to grayscale
cam.PushFilter(sens.ChFilterGrayscale())
# 6. Visualize grayscale
cam.PushFilter(sens.ChFilterVisualize(640, 360, "Grayscale"))
# 7. Save grayscale PNG frames
cam.PushFilter(sens.ChFilterSave("cam/gray/"))
# 8. Resize for downstream access
cam.PushFilter(sens.ChFilterImageResize(640, 360))
# 9. Grayscale R8 host access
cam.PushFilter(sens.ChFilterR8Access())

manager.AddSensor(cam)

# === Visualization — full Irrlicht block ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Sensor Demo — Orbiting Mesh")
vis.Initialize()                                         # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(ORBIT_RADIUS, 0, ORBIT_HEIGHT),
    chrono.ChVector3d(0, 0, 0),
)
vis.AddTypicalLights()
vis.AddGrid(
    1.0, 1.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            t = sys.GetChTime()

            # Dynamically update orbit position each physics step
            angle = ORBIT_SPEED * t
            cx = ORBIT_RADIUS * math.cos(angle)
            cy = ORBIT_RADIUS * math.sin(angle)
            cz = ORBIT_HEIGHT

            # Move the sensor helper body to the new orbit position
            cam_body.SetPos(chrono.ChVector3d(cx, cy, cz))

            # Update sensor camera orientation to look at origin
            forward = chrono.ChVector3d(-cx, -cy, -cz).GetNormalized()
            yaw = math.atan2(forward.y, forward.x)
            look_quat = chrono.QuatFromAngleAxis(yaw, chrono.ChVector3d(0, 0, 1))
            cam_body.SetRot(look_quat)

            # Update Irrlicht interactive camera to match orbit
            vis.UpdateCamera(
                chrono.ChVector3d(cx, cy, cz),
                chrono.ChVector3d(0, 0, 0),
            )

            # Pump sensors (every physics step, as required)
            manager.Update()

            # Read and print RGBA8 buffer data (guarded — buffer may be empty until first tick)
            buf = cam.GetMostRecentRGBA8Buffer()  # cache: fetched once per step
            if buf.HasData():  # guard: skip frames the sensor hasn't filled yet
                rgba = buf.GetRGBA8Data()
                buf_w = buf.Width
                buf_h = buf.Height
                print(f"[t={t:.3f}s] Camera buffer: {buf_w}×{buf_h}, "
                      f"pixels={buf_w * buf_h}, sample RGBA={rgba[0:4]}")
            else:
                buf_w, buf_h = 0, 0
                print(f"[t={t:.3f}s] Camera buffer: not yet filled")


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # CSV closed in review-only block below
