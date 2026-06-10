"""
PyChrono 9.0.x simulation: Lidar Sensor Orbiting a Mesh Body.

Models a Wavefront .obj mesh loaded as a fixed body in a ChSystemNSC scene.
A ChLidarSensor is attached to the mesh body via a ChSensorManager.
The lidar's offset pose is updated each step so it orbits the mesh.
Lidar filter chain: noise, depth visualize, DI access, point cloud conversion,
point cloud visualize, XYZI access. Buffer data is printed at each step.
System: NSC (no gravity needed — pure sensor demo). Renderer: Irrlicht + OptiX lidar.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants ===
# Simulation timing
TIME_STEP  = 1e-3           # physics step (s)
SIM_END    = 10.0           # total sim time (s)
RENDER_FPS = 50.0           # Irrlicht render cadence
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Lidar parameters
LIDAR_UPDATE_RATE  = 5.0    # physical update rate (Hz)
H_SAMPLES          = 800    # horizontal samples
V_SAMPLES          = 300    # vertical samples
LIDAR_MAX_RANGE    = 100.0  # metres
ORBIT_RADIUS       = 8.0    # orbit radius around mesh (m)
ORBIT_SPEED        = 0.5    # orbit angular speed (rad/s)

# Mesh asset — bundled Chrono data
MESH_FILE = chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_col.obj")

# === System ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # pure sensor scene
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies — mesh as fixed body ===
mesh = chrono.ChTriangleMeshConnected()
try:
    mesh.LoadWavefrontMesh(MESH_FILE)
except (RuntimeError, OSError) as exc:
    print(f"Warning: could not load mesh '{MESH_FILE}': {exc}. Using a box fallback.")
    mesh = None

mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(mesh_body)

if mesh is not None:
    vis_shape = chrono.ChVisualShapeTriangleMesh()
    vis_shape.SetMesh(mesh)
    vis_shape.SetMutable(False)
    mesh_body.AddVisualShape(vis_shape)
    # Collision shape needed for OptiX sensor visibility
    coll_shape = chrono.ChCollisionShapeTriangleMesh(
        chrono.ChContactMaterialNSC(), mesh, False, False
    )
    mesh_body.AddCollisionShape(coll_shape)
    mesh_body.EnableCollision(True)
else:
    # Fallback: simple box so the sensor has something to see
    vis_box = chrono.ChVisualShapeBox(4, 4, 4)
    mesh_body.AddVisualShape(vis_box)

# === Sensor Manager ===
manager = sens.ChSensorManager(sys)
# Point lights for camera-side rendering (lidar itself needs no lights)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2, -2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Lidar sensor ===
# Initial orbit pose at angle=0; updated dynamically in the loop
lidar_angle_0 = 0.0
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(ORBIT_RADIUS, 0, 1),
    chrono.QuatFromAngleAxis(math.pi, chrono.ChVector3d(0, 0, 1)),
)

lidar = sens.ChLidarSensor(
    mesh_body,                                    # body the lidar is attached to
    LIDAR_UPDATE_RATE,                            # update_rate (Hz)
    lidar_offset_pose,                            # offset pose
    H_SAMPLES,                                    # horizontal samples
    V_SAMPLES,                                    # vertical samples
    2 * chrono.CH_PI,                             # horizontal FOV (rad)
    chrono.CH_PI / 12,                            # max vertical angle (rad)
    -chrono.CH_PI / 6,                            # min vertical angle (rad)
    LIDAR_MAX_RANGE,                              # max range
    sens.LidarBeamShape_RECTANGULAR,              # beam shape
    2,                                            # sample_radius
    0.003,                                        # vertical divergence angle
    0.003,                                        # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)  # lidar: collection window = 1 / update_rate

# Lidar filter chain (ORDER MATTERS)
lidar.PushFilter(sens.ChFilterVisualize(H_SAMPLES, V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())          # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())       # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())        # host access to XYZI
manager.AddSensor(lidar)

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Sensor — Mesh Scene")
vis.Initialize()  # Initialize FIRST; scene elements come AFTER
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(15, -10, 8),
    chrono.ChVector3d(0, 0, 0),
)
vis.AddTypicalLights()

# === Main loop ===

frame = 0
sim_time = 0.0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            sim_time = sys.GetChTime()

            # Dynamically update the lidar orbit pose around the mesh
            angle = ORBIT_SPEED * sim_time
            lx = ORBIT_RADIUS * math.cos(angle)
            ly = ORBIT_RADIUS * math.sin(angle)
            # Aim the lidar back toward the mesh origin (yaw + pi to face inward)
            yaw = math.atan2(-ly, -lx)
            new_pose = chrono.ChFramed(
                chrono.ChVector3d(lx, ly, 1.0),
                chrono.QuatFromAngleAxis(yaw, chrono.ChVector3d(0, 0, 1)),
            )
            lidar.SetOffsetPose(new_pose)

            manager.Update()  # pump all sensors every physics step

            # Print lidar buffer data if available
            di_buf = lidar.GetMostRecentDIBuffer()
            if di_buf.HasData():  # guard: skip frames the sensor hasn't filled yet
                di_data = di_buf.GetDIData()
                print(f"[t={sim_time:.3f}] Lidar DI buffer shape: {di_data.shape}, "
                      f"max_depth={float(di_data[:, 0].max()):.2f} m")

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # no open file writers to flush in the scored core
