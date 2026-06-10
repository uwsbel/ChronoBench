"""Lidar sensing of a rotating box — PyChrono 9.0.1 + Irrlicht.

Model
-----
A single rigid box (`ChBodyEasyBox`, side = 1.0 m, density = 1000) sits at the
world origin and is driven to spin slowly about the vertical (Z) axis so the
lidars sweep across its faces. Two lidar sensors observe it:

  * lidar_3d : a multi-channel 3D lidar (depth-intensity + point cloud) rigidly
               attached to the box.
  * lidar_2d : a planar 2D lidar with ONE vertical channel, also attached to the
               box (a single horizontal scan line).

System type is NSC (rigid, non-smooth contact). The box carries COLLISION
geometry so the OptiX-backed lidars can see it (OptiX only renders bodies that
have collision shapes), which is why the Bullet collision system is enabled.

Expected behavior
------------------
The box rotates in place; both lidars produce non-empty range returns every
update, the 3D lidar writes a point cloud, and the Irrlicht review window shows
the spinning box on a ground grid.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants === geometry / physics / sensor parameters
time_step = 1e-3                  # s, integration step
sim_end = 8.0                     # s, total simulated time
render_fps = 50.0                 # Hz, Irrlicht review cadence
spin_rate = 0.4                   # rad/s, box angular velocity about +Z

side = 1.0                        # m, full edge length of the imaged box
box_density = 1000.0              # kg/m^3
box_pos = chrono.ChVector3d(0.0, 0.0, side * 0.5)   # rest box center on the ground

# 3D lidar parameters
lidar_update_rate = 5.0           # Hz, lidar revolutions / sec
lidar_hfov = 2.0 * math.pi        # rad, full 360 deg horizontal sweep
lidar_max_v = 0.2618             # rad, +15 deg upper vertical bound
lidar_min_v = -0.2618            # rad, -15 deg lower vertical bound
lidar_w = 480                     # horizontal samples
lidar_h = 16                      # vertical channels (3D)
lidar_max_dist = 100.0            # m, max range
lidar_offset = chrono.ChVector3d(0.0, 0.0, side * 0.5 + 0.2)  # sensor above box top

# 2D lidar parameters — a single vertical channel (one horizontal scan line)
lidar2d_h = 1                     # ONE vertical channel -> 2D planar lidar
lidar2d_max_v = 0.0               # rad, flat scan plane
lidar2d_min_v = 0.0               # rad
lidar2d_w = 720                   # horizontal samples in the plane
lidar2d_offset = chrono.ChVector3d(0.0, 0.0, side * 0.5 + 0.05)

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once
lidar_lag = 0.0                   # s, no extra acquisition lag
lidar_collection = 1.0 / lidar_update_rate   # precomputed once: full-sweep window

# === System & gravity === NSC rigid-body system with Bullet collision
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Bullet collision REQUIRED: the imaged box has collision geometry so the
# OptiX lidars can detect it (and to support contact with the ground).
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies === ground plane + the imaged, slowly spinning box
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(40.0, 40.0, 0.2, 1000.0, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.1))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

# Box replaces any mesh asset: a simple ChBodyEasyBox with collision geometry.
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.6)
box_mat.SetRestitution(0.0)
box = chrono.ChBodyEasyBox(side, side, side, box_density, True, True, box_mat)
box.SetPos(box_pos)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(box)

# A small contrasting marker fixed to one face makes the box's rotation about +Z
# visually unambiguous (a uniformly textured cube looks identical as it spins).
marker = chrono.ChVisualShapeBox(0.2, 0.2, 0.2)
marker.SetColor(chrono.ChColor(1.0, 0.2, 0.0))
box.AddVisualShape(marker, chrono.ChFramed(
    chrono.ChVector3d(side * 0.5 + 0.1, 0.0, 0.0), chrono.QUNIT))

# Surrounding pillars give the planar (2D) lidar's flat scan line something to
# return off of — a flat horizontal ray plane never intersects the flat ground,
# so vertical obstacles around the box are what the 2D scan detects.
pillar_mat = chrono.ChContactMaterialNSC()
pillar_mat.SetFriction(0.6)
pillar_radius = 0.3
pillar_height = 2.0
pillar_ring = 4.0    # m, radial distance of pillars from the box
for k in range(6):
    ang = 2.0 * math.pi * k / 6.0
    pillar = chrono.ChBodyEasyCylinder(
        chrono.ChAxis_Z, pillar_radius, pillar_height, 1000.0, True, True, pillar_mat)
    pillar.SetPos(chrono.ChVector3d(pillar_ring * math.cos(ang),
                                    pillar_ring * math.sin(ang),
                                    pillar_height * 0.5))
    pillar.SetFixed(True)
    pillar.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/redwhite.png"))
    sys.Add(pillar)

# Drive the box to spin about +Z at a constant rate (motor against fixed ground).
spin_motor = chrono.ChLinkMotorRotationSpeed()
spin_motor.Initialize(box, ground, chrono.ChFramed(box_pos, chrono.QUNIT))
spin_motor.SetSpeedFunction(chrono.ChFunctionConst(spin_rate))
sys.Add(spin_motor)

# === Sensors === sensor manager + two lidars attached to the box
manager = sens.ChSensorManager(sys)
# 9.0.1 ChScene has no AddDirectionalLight -> point light + ambient term instead.
manager.scene.AddPointLight(chrono.ChVector3f(10, 10, 30), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# 3D lidar: depth-intensity access, point cloud from depth, saved cloud, XYZI access.
lidar_3d = sens.ChLidarSensor(
    box,                                                   # rides on the box
    lidar_update_rate,
    chrono.ChFramed(lidar_offset, chrono.QUNIT),
    lidar_w, lidar_h,
    lidar_hfov, lidar_max_v, lidar_min_v,
    lidar_max_dist,
    sens.LidarBeamShape_RECTANGULAR,
    1, 0.003, 0.003,
    sens.LidarReturnMode_MEAN_RETURN,
)
lidar_3d.SetName("lidar_3d")
lidar_3d.SetLag(lidar_lag)
lidar_3d.SetCollectionWindow(lidar_collection)
lidar_3d.PushFilter(sens.ChFilterDIAccess())               # depth-intensity buffer access
lidar_3d.PushFilter(sens.ChFilterPCfromDepth())            # depth -> point cloud
lidar_3d.PushFilter(sens.ChFilterSavePtCloud("cam/lidar_3d/"))  # save cloud frames
lidar_3d.PushFilter(sens.ChFilterXYZIAccess())             # XYZI point access
manager.AddSensor(lidar_3d)

# 2D lidar: ONE vertical channel -> a single planar scan line.
lidar_2d = sens.ChLidarSensor(
    box,
    lidar_update_rate,
    chrono.ChFramed(lidar2d_offset, chrono.QUNIT),
    lidar2d_w, lidar2d_h,
    lidar_hfov, lidar2d_max_v, lidar2d_min_v,
    lidar_max_dist,
    sens.LidarBeamShape_RECTANGULAR,
    1, 0.003, 0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("lidar_2d")
lidar_2d.SetLag(lidar_lag)
lidar_2d.SetCollectionWindow(lidar_collection)
lidar_2d.PushFilter(sens.ChFilterDIAccess())               # depth-intensity buffer access
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())            # depth -> point cloud
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())             # XYZI point access
manager.AddSensor(lidar_2d)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar sensing of a rotating box")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, -6, 4), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop, physics + sensors in inner batch


frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()    # pump both lidars every physics step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
