"""Lidar scanning of a static triangular mesh in PyChrono (Irrlicht + OptiX sensor).

Model
-----
A single rigid body holds a Wavefront `.obj` triangular mesh (the HMMWV chassis
shell from the Chrono data set) and is fixed in the world. The body carries BOTH
a visual triangle mesh (for the Irrlicht review window) AND a collision triangle
mesh, because the OptiX-based lidar only returns hits off geometry that exists in
the collision/ray-traced scene tree.

System type
-----------
`ChSystemNSC` (rigid, non-smooth contact). The scene has collision geometry on
the meshed body, so the Bullet collision system is enabled explicitly.

Sensor
------
A `ChLidarSensor` rides on a separate, kinematically driven "carrier" body. Each
step the carrier is repositioned on a circular orbit around the mesh and the
lidar is re-aimed at the mesh center, so the device sweeps the object from every
azimuth. The lidar filter chain converts range/intensity to a point cloud, gives
access to the depth/intensity (DI) and Cartesian (XYZI) buffers, and saves each
scan as a point-cloud file. The depth buffer is read every step to report the
live valid-return range.

Expected behavior
------------------
The carrier orbits the fixed mesh for the whole run. The lidar returns a stable
band of finite ranges (roughly the orbit radius minus the object half-width) with
a few "miss" rays at the silhouette edges, and the reported min/mean hit distance
tracks the orbit geometry. The Irrlicht window shows the static mesh with the
carrier circling it.
"""

# === Imports ===
import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Named constants === geometry / physics / sensor configuration
time_step = 4e-3                       # s; raised to keep the OptiX render under the wall-clock budget
sim_end = 12.0                         # s; long enough for a full orbit of the mesh
render_fps = 30.0                      # Hz; Irrlicht review-frame cadence

mesh_file = "vehicle/hmmwv/hmmwv_chassis.obj"   # triangular mesh visualized as the fixed body
mesh_center = chrono.ChVector3d(0.0, 0.0, 0.0)  # the mesh body sits at the world origin

orbit_radius = 8.0                     # m; lidar carrier orbit radius about the mesh
orbit_height = 1.5                     # m; lidar height above ground while orbiting
orbit_period = 8.0                     # s; time for one full revolution
orbit_omega = 2.0 * math.pi / orbit_period   # rad/s; precomputed once, reused every step

lidar_update_rate = 1.0 / time_step    # Hz; one scan per physics step
lidar_w = 360                          # horizontal samples (1 deg azimuth resolution)
lidar_h = 16                           # vertical channels
lidar_hfov = 2.0 * math.pi             # rad; full 360 deg horizontal field of view
lidar_max_vert = 0.2618                # rad; +15 deg top channel
lidar_min_vert = -0.2618               # rad; -15 deg bottom channel
lidar_max_dist = 40.0                  # m; maximum sensing range
lidar_sample_radius = 2                # super-sampling radius (beam footprint averaging)
lidar_div_angle = 0.003                # rad; beam divergence (vertical and horizontal)

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once: physics steps per frame

# === System & gravity === NSC rigid system; Bullet collision needed for lidar returns
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Collision geometry feeds the OptiX ray tracer; without Bullet the lidar sees nothing.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies === fixed triangular mesh (visual + collision) plus the lidar carrier
mesh_path = chrono.GetChronoDataFile(mesh_file)

trimesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(mesh_path, True, True)

mesh_body = chrono.ChBody()
mesh_body.SetPos(mesh_center)
mesh_body.SetFixed(True)                       # static scene object: never integrated

# Visual triangle mesh for the Irrlicht review window.
vis_mesh = chrono.ChVisualShapeTriangleMesh()
vis_mesh.SetMesh(trimesh)
vis_mesh.SetName("scanned_mesh")
vis_mesh.SetMutable(False)
mesh_body.AddVisualShape(vis_mesh, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# Collision triangle mesh so the lidar's rays actually strike the object.
mesh_mat = chrono.ChContactMaterialNSC()       # NSC material to match the NSC system
mesh_mat.SetFriction(0.6)
mesh_mat.SetRestitution(0.0)
coll_mesh = chrono.ChCollisionShapeTriangleMesh(mesh_mat, trimesh, True, True, 0.005)
mesh_body.AddCollisionShape(coll_mesh, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
mesh_body.EnableCollision(True)
sys.Add(mesh_body)

# Ground plane gives the orbiting lidar a floor reference and extra return surface.
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(60.0, 60.0, 0.2, 1000.0, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
sys.Add(ground)

# Lidar carrier: a fixed-but-repositioned body the sensor rides on while orbiting.
lidar_carrier = chrono.ChBody()
lidar_carrier.SetPos(chrono.ChVector3d(orbit_radius, 0.0, orbit_height))
lidar_carrier.SetFixed(True)                   # driven kinematically by SetPos/SetRot each step
sys.Add(lidar_carrier)

# === Sensor === lidar manager, scene lighting, lidar + standard filter chain
manager = sens.ChSensorManager(sys)
# 9.0.1 ChScene exposes no directional light: use a point light + ambient term.
manager.scene.AddPointLight(chrono.ChVector3f(20, 20, 40), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

lidar = sens.ChLidarSensor(
    lidar_carrier,                                       # body the lidar rides on
    lidar_update_rate,                                   # Hz
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),         # offset on the carrier (re-aimed via carrier rot)
    lidar_w, lidar_h,                                    # horizontal / vertical samples
    lidar_hfov,                                          # horizontal FOV (rad)
    lidar_max_vert, lidar_min_vert,                      # vertical FOV bounds (rad)
    lidar_max_dist,                                      # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    lidar_sample_radius,                                 # super-sampling radius
    lidar_div_angle, lidar_div_angle,                    # vertical / horizontal divergence (rad)
    sens.LidarReturnMode_MEAN_RETURN,
)
lidar.SetName("orbit_lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)

# Standard lidar filter chain: raw depth/intensity -> point cloud -> accessors + save.
lidar.PushFilter(sens.ChFilterDIAccess())                # access depth+intensity buffer
lidar.PushFilter(sens.ChFilterPCfromDepth())             # convert depth scan to a 3D point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())              # access Cartesian point cloud buffer
lidar.PushFilter(sens.ChFilterSavePtCloud("cam/ptcloud/"))   # persist each scan to disk
manager.AddSensor(lidar)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar scanning a fixed triangular mesh")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(14, -14, 8), mesh_center)
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 30, 30, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.4), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === orbit the lidar, pump the sensor manager, report live ranges

up = chrono.ChVector3d(0, 0, 1)            # world up, used to aim the carrier at the mesh
lidar_local_fwd = chrono.ChVector3d(1, 0, 0)   # cache: lidar looks along carrier +X; precomputed once



frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            # Drive the carrier around the mesh and re-aim its +X at the mesh center.
            angle = orbit_omega * t
            eye = chrono.ChVector3d(orbit_radius * math.cos(angle),
                                    orbit_radius * math.sin(angle),
                                    orbit_height)
            lidar_carrier.SetPos(eye)
            forward = (mesh_center - eye).GetNormalized()
            aim_quat = chrono.QuatFromVec2Vec(lidar_local_fwd, forward)
            lidar_carrier.SetRot(aim_quat)

            manager.Update()        # pump the lidar every physics step

            # Read the depth/intensity buffer and report live valid ranges.
            di_buf = lidar.GetMostRecentDIBuffer()   # may be empty before the lidar's first tick
            num_hits = 0
            min_range = float("nan")
            mean_range = float("nan")
            if di_buf.HasData():                     # guard: skip ticks the lidar has not filled yet
                di = di_buf.GetDIData()              # (h, w, 2): [:, :, 0] = range, [:, :, 1] = intensity
                ranges = di[:, :, 0]
                valid = ranges[(ranges > 0.0) & (ranges < lidar_max_dist)]
                num_hits = int(valid.size)
                if num_hits > 0:
                    min_range = float(valid.min())
                    mean_range = float(valid.mean())
                print(f"t={t:6.3f}s  lidar hits={num_hits:5d}  "
                      f"min={min_range:7.3f} m  mean={mean_range:7.3f} m")


            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review + sensor videos, plot the log, drop frames
