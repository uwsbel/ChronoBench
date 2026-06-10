"""
Lidar Demo — Box Object with 3D and 2D Lidar Sensors.

System type: ChSystemNSC (NSC rigid-body with collision for box and floor).
Bodies: a fixed ground floor, a rotating box with texture.
Sensors: one 3D lidar sensor and one additional 2D lidar sensor (1 vertical channel),
both attached to the rotating box and managed by a ChSensorManager.
Expected behavior: the rotating box is sensed by both lidar sensors;
the 3D lidar produces a volumetric point cloud, and the 2D lidar produces a
single horizontal sweep, both visualized in real time.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants ===
time_step = 1e-3          # physics time step [s]
sim_end   = 10.0          # simulation end time [s]
render_fps = 50.0         # Irrlicht render rate [Hz]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

side = 1.0                # box side length [m]
box_density = 1000.0      # box material density [kg/m³]
box_pos = chrono.ChVector3d(0, 1.0, 0)  # box center, Y-up

lidar_offset_x = -12.0    # lidar offset behind the box [m]
lidar_offset_z =  1.0     # lidar height [m]

# === System & gravity (NSC — contact between box and floor) ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

# === Bodies ===

# Fixed ground floor
floor = chrono.ChBodyEasyBox(20.0, 0.5, 20.0, 1000.0, True, True, mat)
floor.SetPos(chrono.ChVector3d(0, -0.25, 0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# Box body (replaces the mesh — ChBodyEasyBox with collision enabled)
box_body = chrono.ChBodyEasyBox(side, side, side, box_density, True, True, mat)
box_body.SetPos(box_pos)
box_body.SetAngVelParent(chrono.ChVector3d(0, 0.5, 0.1))  # gentle tumble
box_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(box_body)

# === Sensor Manager ===
manager = sens.ChSensorManager(sys)
# Point lights for sensor scene (camera-only lighting; lighted for any cam if added later)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 100.0),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === 3D Lidar Sensor (attached to box) ===
h_samples_3d = 800
v_samples_3d = 300
offset_pose_3d = chrono.ChFramed(
    chrono.ChVector3d(lidar_offset_x, 0, lidar_offset_z),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar_3d = sens.ChLidarSensor(
    box_body,                                     # attach to the box
    5.0,                                          # update_rate (Hz) — physical rate
    offset_pose_3d,
    h_samples_3d,                                 # horizontal samples
    v_samples_3d,                                 # vertical samples
    2 * chrono.CH_PI,                             # horizontal FOV (full 360°)
    chrono.CH_PI / 12,                            # max vertical angle [rad]
    -chrono.CH_PI / 6,                            # min vertical angle [rad]
    100.0,                                        # max range [m]
    sens.LidarBeamShape_RECTANGULAR,
    2,                                            # sample radius
    0.003,                                        # vertical divergence angle
    0.003,                                        # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_3d.SetName("3D Lidar Sensor")
lidar_3d.SetLag(0)
lidar_3d.SetCollectionWindow(1.0 / 5.0)   # collection window = 1 / update_rate

# 3D lidar filter chain
lidar_3d.PushFilter(sens.ChFilterVisualize(h_samples_3d, v_samples_3d, "Raw 3D Lidar Depth"))
lidar_3d.PushFilter(sens.ChFilterDIAccess())                   # host access to depth+intensity
lidar_3d.PushFilter(sens.ChFilterPCfromDepth())                # depth -> XYZI point cloud
lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
lidar_3d.PushFilter(sens.ChFilterXYZIAccess())                 # host access to XYZI point cloud
manager.AddSensor(lidar_3d)

# === 2D Lidar Sensor (1 vertical channel — attached to box) ===
h_samples_2d = 800
offset_pose_2d = chrono.ChFramed(
    chrono.ChVector3d(lidar_offset_x, 0, lidar_offset_z + 0.1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar_2d = sens.ChLidarSensor(
    box_body,                                     # attach to the box
    5.0,                                          # update_rate (Hz) — physical rate
    offset_pose_2d,
    h_samples_2d,                                 # horizontal samples
    1,                                            # v_samples = 1 → 2D lidar (single sweep)
    2 * chrono.CH_PI,                             # horizontal FOV (full 360°)
    0,                                            # max_vert_angle = 0 for 2D
    0,                                            # min_vert_angle = 0 for 2D
    100.0,                                        # max range [m]
    sens.LidarBeamShape_RECTANGULAR,
    2,                                            # sample radius
    0.003,                                        # vertical divergence angle
    0.003,                                        # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)
lidar_2d.SetCollectionWindow(1.0 / 5.0)   # collection window = 1 / update_rate

# 2D lidar filter chain
lidar_2d.PushFilter(sens.ChFilterVisualize(h_samples_2d, 1, "Raw 2D Lidar Depth"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())                   # host access to depth+intensity
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())                # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())                 # host access to XYZI
manager.AddSensor(lidar_2d)

# === Visualization — full Irrlicht scene ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Demo — Box with 3D and 2D Lidar")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                        # Initialize FIRST (Irrlicht)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-5, 3, -8), chrono.ChVector3d(0, 1, 0))  # AFTER Initialize
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 40, 40,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only recording setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()               # pump sensors every physics step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
