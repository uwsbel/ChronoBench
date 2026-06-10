"""
Lidar scanning of a fixed box body with two lidar sensors.

A box body (ChBodyEasyBox) replaces the triangular mesh from the previous turn.
The first lidar (3D, 800 horizontal samples, 1 vertical channel) is attached to
the box and orbits around it. A second 2D lidar (800 horizontal samples, 1
vertical sample, single scan plane) is also attached to the box with its own
offset. Demonstrates: ChBodyEasyBox creation, dual lidar sensors with distinct
filter chains, and per-step buffer printing.
"""

import os, math, csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Box geometry
box_side = 1.0   # metres (cube)

# Lidar orbit parameters
orbit_radius = 3.0     # metres from box centre
orbit_speed = 0.5      # rad/s

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Box body (fixed, ChBodyEasyBox) ===
body = chrono.ChBodyEasyBox(box_side, box_side, box_side, 1000)
body.SetFixed(True)
body.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(body)

# === Contact material (required for collision) ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)
mat.SetRestitution(0.0)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)

# Add scene lights for the sensor renderer (point lights spread across scene)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Lidar sensor 1 — 3D lidar attached to the box ===
orbit_angle = [0.0]   # mutable container so the loop can update it

lidar_offset_base = chrono.ChFramed(
    chrono.ChVector3d(orbit_radius, 0, 0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

lidar = sens.ChLidarSensor(
    body,
    5.0,                                 # update_rate Hz (physical)
    lidar_offset_base,
    800,                                 # horizontal_samples
    1,                                   # vertical_samples (2D scan plane)
    2 * chrono.CH_PI,                    # horizontal_fov rad
    0.0,                                 # max_vert_angle
    0.0,                                 # min_vert_angle
    100.0,                               # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                                   # sample_radius
    0.003,                               # vert_divergence_angle
    0.003,                               # hori_divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)

# Lidar filter chain (ORDER MATTERS):
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())                      # depth + intensity host access
lidar.PushFilter(sens.ChFilterPCfromDepth())                   # XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())                    # XYZI host access
manager.AddSensor(lidar)

# === Lidar sensor 2 — 2D lidar with one vertical channel ===
# Offset on the opposite side of the box from lidar 1
lidar_2d_offset = chrono.ChFramed(
    chrono.ChVector3d(-orbit_radius, 0, 0.5),
    chrono.QuatFromAngleAxis(math.pi, chrono.ChVector3d(0, 1, 0)),
)

lidar_2d = sens.ChLidarSensor(
    body,
    5.0,                                 # update_rate Hz (physical)
    lidar_2d_offset,
    800,                                 # horizontal_samples
    1,                                   # vertical_samples (2D = 1 channel)
    2 * chrono.CH_PI,                    # horizontal_fov rad
    0.0,                                 # max_vert_angle
    0.0,                                 # min_vert_angle
    100.0,                               # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                                   # sample_radius
    0.003,                               # vert_divergence_angle
    0.003,                               # hori_divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("Lidar 2D Sensor")
lidar_2d.SetLag(0)
lidar_2d.SetCollectionWindow(1.0 / 5.0)

# 2D lidar filter chain
lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "2D Lidar Depth"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())                  # depth + intensity host access
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())               # XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())                # XYZI host access
manager.AddSensor(lidar_2d)

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Box Scan")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, -5, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === CSV logging (review-only) ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))
csv_path = "simulation_data.csv"
csv_file = None
csv_writer = None
if REC:
    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=["time", "lidar_angle", "buffer_has_data"])
    csv_writer.writeheader()

# === Main loop ===
frame = 0
os.makedirs("frames", exist_ok=True)

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # --- update lidar 1 orbit each frame ---
        orbit_angle[0] += orbit_speed * time_step * render_every
        new_offset = chrono.ChFramed(
            chrono.ChVector3d(
                orbit_radius * math.cos(orbit_angle[0]),
                orbit_radius * math.sin(orbit_angle[0]),
                0,
            ),
            chrono.QuatFromAngleAxis(orbit_angle[0], chrono.ChVector3d(0, 1, 0)),
        )
        lidar.SetOffsetPose(new_offset)

        # --- per-frame capture (review-only) ---
        if REC:
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")
        frame += 1

        # --- inner physics batch ---
        for _ in range(render_every):
            manager.Update()   # update all sensors every physics step

            # Print lidar 1 buffer data each step
            di_buf = lidar.GetMostRecentDIBuffer()
            if di_buf.HasData():
                di_data = di_buf.GetDIData()   # shape: (H, W, 2) -> [depth, intensity]
                depths = di_data[..., 0].flatten()
                intensities = di_data[..., 1].flatten()
                n = depths.size
                print(f"  t={sys.GetChTime():.3f}  lidar_angle={orbit_angle[0]:.3f}  "
                      f"points={n}  range_min={depths.min():.3f}  range_max={depths.max():.3f}")

                if REC:
                    csv_writer.writerow({
                        "time": sys.GetChTime(),
                        "lidar_angle": orbit_angle[0],
                        "buffer_has_data": True,
                    })

            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

finally:
    if REC and csv_file:
        csv_file.close()
        import sim_recording as rec
        rec.assemble_all_videos("frames", sensor_dirs=[])
        rec.cleanup_frames("frames")
