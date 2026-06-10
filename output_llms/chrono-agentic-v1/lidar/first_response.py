"""
Lidar sensor simulation — static triangular mesh (Wavefront .obj) visualized as a fixed body,
with a ChLidarSensor orbiting around it via dynamic pose updates each simulation step.

System: ChSystemNSC (no gravity — purely static scene with lidar scanning).
Bodies: one fixed mesh body loaded from lime_bunny.obj (bundled Chrono asset), using
        ChBodyEasyMesh with create_collision=True so the OptiX lidar sensor registers
        the geometry. Visual scale is set via ChVisualShapeModelFile.SetScale (x10 → ~1.5 m tall).
Sensors: ChLidarSensor with noise model (ChNoiseNormal), visualization
         (ChFilterVisualize depth + ChFilterVisualizePointCloud), access
         (ChFilterDIAccess + ChFilterXYZIAccess), and point-cloud saving
         (ChFilterSavePtCloud). Lidar orbits the mesh body each sim step.
Expected behavior: Irrlicht window shows the mesh; OptiX lidar preview shows depth/point-cloud;
         lidar orbits at 3 m radius; buffer data (range, intensity) printed at each step.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Simulation parameters ===
time_step = 1e-3       # physics step size (s)
sim_end   = 10.0       # simulation end time (s)
render_fps = 50.0      # Irrlicht frames per second for review video

# Lidar orbit parameters
orbit_radius   = 3.0   # orbit radius around the mesh (m)
orbit_altitude = 1.0   # orbit altitude (m)
orbit_speed    = 0.5   # angular speed of the orbit (rad/s)

# Lidar sensor physical parameters
lidar_update_rate  = 5.0        # Hz — physical rate (not 1/dt)
h_samples          = 800        # horizontal samples
v_samples          = 300        # vertical samples
h_fov              = 2 * chrono.CH_PI  # full 360° horizontal sweep
max_vert_angle     =  chrono.CH_PI / 12  # ~15 deg up
min_vert_angle     = -chrono.CH_PI / 6   # ~30 deg down
max_range          = 100.0       # metres

# Precomputed render cadence — one Irrlicht frame per render_every physics steps
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # static scene — no gravity
# Collision system required because ChBodyEasyMesh with create_collision=True adds a shape
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies — fixed mesh from a Wavefront .obj file ===
# Contact material for the collision mesh (NSC matches ChSystemNSC)
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)

# ChBodyEasyMesh with create_collision=True registers geometry for OptiX (lidar sensor)
mesh_path = chrono.GetChronoDataFile("models/lime_bunny.obj")
mesh_body = chrono.ChBodyEasyMesh(
    mesh_path,   # Wavefront .obj path
    1000.0,      # density (kg/m³)
    True,        # compute_mass
    True,        # create_visualization — adds ChVisualShapeModelFile automatically
    True,        # create_collision — adds ChCollisionShapeTriangleMesh for OptiX registration
    mat,         # contact material (NSC)
)
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))

# Scale the visual model so the ~15 cm bunny renders at ~1.5 m scale in Irrlicht
vis_shape = mesh_body.GetVisualShape(0)
if vis_shape is not None:
    try:
        vis_shape.SetScale(10.0)   # scale: ~15 cm → ~1.5 m visible in the window
    except AttributeError:
        pass  # if SetScale is unsupported on this shape type, leave natural scale

sys.AddBody(mesh_body)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)
# Point lights for the OptiX sensor rendering environment
manager.scene.AddPointLight(
    chrono.ChVector3f(5, 5, 10),
    chrono.ChColor(1, 1, 1),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-5, -5, 10),
    chrono.ChColor(1, 1, 1),
    300.0,
)

# === Lidar sensor — orbiting the mesh body ===
# Initial orbit position at angle = 0 (along +X axis)
init_angle = 0.0
init_x     = orbit_radius * math.cos(init_angle)
init_y     = orbit_radius * math.sin(init_angle)
init_z     = orbit_altitude

# Offset pose on the body: lidar at orbit position, yaw to face mesh center (origin)
init_yaw = math.atan2(-init_y, -init_x)  # yaw angle to face origin from orbit point
init_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(init_x, init_y, init_z),
    chrono.QuatFromAngleAxis(init_yaw, chrono.ChVector3d(0, 0, 1)),
)

lidar = sens.ChLidarSensor(
    mesh_body,          # attached to the (fixed) mesh body
    lidar_update_rate,  # update_rate Hz — physical rate (not 1/dt)
    init_offset_pose,   # initial offset pose on the body
    h_samples,          # horizontal samples
    v_samples,          # vertical samples
    h_fov,              # horizontal FOV (rad)
    max_vert_angle,     # max vertical angle (rad)
    min_vert_angle,     # min vertical angle (rad)
    max_range,          # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                  # sample_radius
    0.003,              # vert divergence_angle
    0.003,              # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Orbiting Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)  # lidar: 1 / update_rate

# --- Lidar filter chain (ORDER MATTERS — each filter snapshots buffer at its position) ---
# 1. Visualize raw depth image in an OptiX preview window
lidar.PushFilter(sens.ChFilterVisualize(h_samples, v_samples, "Raw Lidar Depth"))
# 2. Host access to raw DI (depth+intensity) buffer — guarded with HasData() below
lidar.PushFilter(sens.ChFilterDIAccess())
# 3. Convert depth/intensity map to XYZI point cloud
lidar.PushFilter(sens.ChFilterPCfromDepth())
# 4. Visualize point cloud in an OptiX preview window
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
# 5. Host access to XYZI buffer for reading/printing
lidar.PushFilter(sens.ChFilterXYZIAccess())
# 6. Save point cloud data to disk (saving option)
lidar.PushFilter(sens.ChFilterSavePtCloud("cam/lidar_ptcloud/"))

manager.AddSensor(lidar)

# === Visualization — full Irrlicht block ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Sensor — Orbiting Mesh")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()                                                  # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, -4, 3), chrono.ChVector3d(0, 0, 0.5))  # AFTER Initialize
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===

# === Main simulation loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()

            # --- Dynamically update lidar orbit position each sim step ---
            angle   = orbit_speed * t  # current orbit angle (rad)
            lx      = orbit_radius * math.cos(angle)
            ly      = orbit_radius * math.sin(angle)
            lz      = orbit_altitude
            yaw_dir = math.atan2(-ly, -lx)  # face toward mesh origin each step

            new_pose = chrono.ChFramed(
                chrono.ChVector3d(lx, ly, lz),
                chrono.QuatFromAngleAxis(yaw_dir, chrono.ChVector3d(0, 0, 1)),
            )
            lidar.SetOffsetPose(new_pose)  # orbit update applied every step

            # --- Update all sensors (exactly once per physics step) ---
            manager.Update()

            # --- Print lidar DI buffer data at each sim step if available ---
            di_buf = lidar.GetMostRecentDIBuffer()  # may be empty before first sensor tick
            if di_buf.HasData():                     # guard: safe to read only after HasData
                di_data = di_buf.GetDIData()
                # di_data: float32 ndarray (v_samples, h_samples, 2) → [..., 0]=range, [..., 1]=intensity
                center_range     = float(di_data[v_samples // 2, h_samples // 2, 0])
                center_intensity = float(di_data[v_samples // 2, h_samples // 2, 1])
                print(
                    f"t={t:.3f}s | orbit angle={math.degrees(angle):.1f}° | "
                    f"DI buf {di_buf.Width}×{di_buf.Height} | "
                    f"center ray: range={center_range:.3f}m intensity={center_intensity:.3f}"
                )


            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # flush / teardown — writers closed below in review-only block
