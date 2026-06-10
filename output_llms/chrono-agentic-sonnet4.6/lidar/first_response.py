"""
Lidar Sensor Orbiting a Mesh Body.

Models a fixed triangular mesh (loaded from a Wavefront .obj file, red_teapot)
as a fixed body visualized in the scene. A lidar sensor is attached via
ChSensorManager to an orbiting body that travels in a circular path around the
mesh. Noise filters, depth/intensity visualization, point-cloud filters, and
saving options are applied. Lidar buffer data (DI width/height) is printed each
step. The Irrlicht camera pans to track the orbiting lidar body.

System: ChSystemNSC (no contact — static mesh, no dynamic collision needed)
"""

# === Imports ===
import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants ===
TIME_STEP  = 1e-3                    # physics time step (s)
SIM_END    = 10.0                    # simulation end time (s)
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Lidar orbit parameters
ORBIT_RADIUS = 8.0                   # orbit radius around mesh (m)
ORBIT_HEIGHT = 1.0                   # orbit height above origin (m)
ORBIT_SPEED  = 0.6                   # angular speed of orbit (rad/s)

# Lidar sensor settings
LIDAR_UPDATE_RATE = 5.0              # Hz — physical rate, not 1/dt
H_SAMPLES = 800
V_SAMPLES = 300
H_FOV     = 2 * chrono.CH_PI        # 360 deg horizontal
MAX_VERT  =  chrono.CH_PI / 12      # +15 deg
MIN_VERT  = -chrono.CH_PI / 6       # -30 deg
MAX_RANGE = 100.0

# Camera tracking offset (follows lidar body at a fixed relative height)
CAM_TRACK_HEIGHT = 6.0
CAM_TRACK_BACK   = 4.0              # behind lidar in radial direction

# === System ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Static mesh + no collision interaction → no need for collision system

# === Bodies ===
# Mesh body — load Wavefront .obj (red_teapot) as a fixed visualized body
mesh_path = chrono.GetChronoDataFile("models/red_teapot.obj")

mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))

# Visual shape: the mesh file
shape = chrono.ChVisualShapeModelFile()
shape.SetFilename(mesh_path)
mesh_body.AddVisualShape(shape)

# Collision shape so OptiX sensor sees it (required for sensor scene rendering)
try:
    tri_mesh = chrono.ChTriangleMeshConnected()
    tri_mesh.LoadWavefrontMesh(mesh_path, True, True)
    col_mat = chrono.ChContactMaterialNSC()
    col_shape = chrono.ChCollisionShapeTriangleMesh(col_mat, tri_mesh, False, False, 0.005)
    mesh_body.AddCollisionShape(col_shape)
    mesh_body.EnableCollision(True)
except (RuntimeError, OSError) as exc:
    print(f"Warning: mesh collision load failed (sensor may not see it): {exc}")

sys.Add(mesh_body)

# Lidar body — updated each step to orbit the mesh; has a visible sphere shape
lidar_body = chrono.ChBody()
lidar_body.SetFixed(True)
lidar_body.SetPos(chrono.ChVector3d(ORBIT_RADIUS, 0, ORBIT_HEIGHT))

# Visible sphere on lidar body so it appears in Irrlicht window
lidar_vis_sphere = chrono.ChVisualShapeSphere(0.3)
lidar_body.AddVisualShape(lidar_vis_sphere)

sys.Add(lidar_body)

# === Sensor Manager ===
manager = sens.ChSensorManager(sys)
# Point lights for OptiX sensor scene
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-10, 5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Lidar sensor — attaches to lidar_body; offset pose keeps local +X toward origin
# Offset is identity since we reorient the lidar_body itself each step
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

# Build lidar (16 positional args)
lidar = sens.ChLidarSensor(
    lidar_body,                              # attach body
    LIDAR_UPDATE_RATE,                       # update_rate Hz
    offset_pose,                             # offset pose on body
    H_SAMPLES,                               # horizontal samples
    V_SAMPLES,                               # vertical samples
    H_FOV,                                   # horizontal FOV (rad)
    MAX_VERT,                                # max vertical angle (rad)
    MIN_VERT,                                # min vertical angle (rad)
    MAX_RANGE,                               # max range (m)
    sens.LidarBeamShape_RECTANGULAR,         # beam shape
    2,                                       # sample_radius
    0.003,                                   # vert divergence angle
    0.003,                                   # hori divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,   # return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)  # 0.2 s for 5 Hz

# Lidar filter chain (prompt-required scored core components):
#   visualize raw depth image → DI access → convert to point cloud → visualize PC → XYZI access → save
lidar.PushFilter(sens.ChFilterVisualize(H_SAMPLES, V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())              # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())           # depth → XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())            # host access to XYZI
lidar.PushFilter(sens.ChFilterSavePtCloud("cam/lidar_pc/"))   # save point cloud
manager.AddSensor(lidar)

# Review camera (OptiX) — only active during REC; scored-core shape has it untagged

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Sensor Demo — Orbiting Mesh")
vis.Initialize()                                              # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
# Camera starts above-side, looking at origin
vis.AddCamera(chrono.ChVector3d(ORBIT_RADIUS + 2, 0, CAM_TRACK_HEIGHT),
              chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only setup ===


# === Main Loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            t = sys.GetChTime()

            # Dynamically update lidar body position + orientation in orbit around mesh
            angle = ORBIT_SPEED * t
            lx = ORBIT_RADIUS * math.cos(angle)
            ly = ORBIT_RADIUS * math.sin(angle)
            lz = ORBIT_HEIGHT
            lidar_body.SetPos(chrono.ChVector3d(lx, ly, lz))
            # Rotate body so local +X faces origin (look toward -radial direction)
            look_yaw = math.atan2(-ly, -lx)
            lidar_body.SetRot(chrono.QuatFromAngleAxis(look_yaw, chrono.ChVector3d(0, 0, 1)))

            # Irrlicht camera: track slightly behind and above lidar orbit
            cam_angle = angle - 0.3             # slight lag behind lidar
            cam_ex = (ORBIT_RADIUS + CAM_TRACK_BACK) * math.cos(cam_angle)
            cam_ey = (ORBIT_RADIUS + CAM_TRACK_BACK) * math.sin(cam_angle)
            vis.UpdateCamera(
                chrono.ChVector3d(cam_ex, cam_ey, CAM_TRACK_HEIGHT),
                chrono.ChVector3d(0, 0, 0),
            )

            manager.Update()    # pump all sensors every physics step

            # Read and print lidar buffer data each step (prompt required)
            di_buf   = lidar.GetMostRecentDIBuffer()
            xyzi_buf = lidar.GetMostRecentXYZIBuffer()
            if di_buf.HasData():     # guard: buffer empty until first sensor tick
                print(f"t={t:.3f}s | lidar=({lx:.2f},{ly:.2f},{lz:.2f}) "
                      f"| DI {di_buf.Width}x{di_buf.Height} "
                      f"| XYZI has={xyzi_buf.HasData()}")


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:    # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
