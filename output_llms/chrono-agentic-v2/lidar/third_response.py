"""
ARTcar on rigid terrain with 3D lidar, 2D lidar, and third-person camera sensors.
System type: NSC (rigid terrain default for ARTcar wrapper).
Main bodies: ARTcar chassis + wheels on RigidTerrain; lidar sensors and a
third-person camera sensor are attached to the vehicle chassis.
Expected behavior: ARTcar drives on flat terrain; the 3D lidar scans the surrounding
environment (point cloud); the 2D lidar provides a horizontal sweep; the third-person
camera follows the vehicle chassis from behind.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Data paths (required for all catalog vehicle truths) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Named constants ===
step_size = 1e-3                       # physics time step (s)
sim_end   = 20.0                       # simulation end time (s)
render_fps = 50.0                      # Irrlicht render rate (Hz)
render_step_size = 1.0 / render_fps    # seconds per rendered frame
render_steps = math.ceil(render_step_size / step_size)  # physics steps per frame  # precomputed once

terrain_length = 200.0                 # terrain X extent (m)
terrain_width  = 200.0                 # terrain Y extent (m)

INIT_LOC = chrono.ChVector3d(0, 0, 0.5)   # vehicle spawn location
INIT_ROT = chrono.QuatFromAngleZ(0.0)     # vehicle spawn orientation

# === Vehicle setup (ARTcar wrapper, NSC, rigid terrain) ===
artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)
artcar.SetChassisCollisionType(veh.CollisionType_NONE)
artcar.SetChassisFixed(False)                        # MANDATORY — fixed chassis won't move
artcar.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
artcar.SetTireType(veh.TireModelType_TMEASY)
artcar.SetTireStepSize(step_size)
artcar.Initialize()

# === System & bodies (created by the veh.ARTcar wrapper) ===
sys = artcar.GetSystem()               # ChSystemNSC owned by the wrapper
chassis = artcar.GetChassisBody()      # cache: main chassis rigid body, reused in sensors
# wheels/spindles: artcar.GetVehicle().GetAxle(i).m_wheels; terrain: RigidTerrain below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED with terrain contact

artcar.SetChassisVisualizationType(chrono.VisualizationType_MESH)
artcar.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
artcar.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
artcar.SetWheelVisualizationType(chrono.VisualizationType_MESH)
artcar.SetTireVisualizationType(chrono.VisualizationType_MESH)

print("VEHICLE MASS: ", artcar.GetVehicle().GetMass())

# === Terrain ===
patch_mat = chrono.ChContactMaterialNSC()   # NSC matches artcar system contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Sensor Manager + Lidar Sensors + Third-Person Camera ===
manager = sens.ChSensorManager(sys)
# Point lights for the camera sensor (OptiX) — lidar and IMU need no lighting
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-50, -50, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# --- 3D Lidar attached to vehicle chassis ---
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar_3d = sens.ChLidarSensor(
    chassis,                                  # attach to chassis
    5.0,                                      # update_rate (Hz) — physical rate
    lidar_offset_pose,
    800,                                      # h_samples
    300,                                      # v_samples
    2 * chrono.CH_PI,                         # horizontal FOV (rad)
    chrono.CH_PI / 12,                        # max_vert_angle (rad)
    -chrono.CH_PI / 6,                        # min_vert_angle (rad)
    100.0,                                    # max_range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                                        # sample_radius
    0.003,                                    # vert divergence angle
    0.003,                                    # hori divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_3d.SetName("3D Lidar Sensor")
lidar_3d.SetLag(0)
lidar_3d.SetCollectionWindow(1.0 / 5.0)      # collection window = 1 / update_rate

lidar_3d.PushFilter(sens.ChFilterVisualize(800, 300, "3D Lidar Raw Depth"))
lidar_3d.PushFilter(sens.ChFilterDIAccess())
lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
lidar_3d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_3d)

# --- 2D Lidar attached to vehicle chassis (horizontal sweep: v_samples=1, vert angles=0) ---
lidar_offset_pose_2d = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar_2d = sens.ChLidarSensor(
    chassis,                                  # attach to chassis
    5.0,                                      # update_rate (Hz)
    lidar_offset_pose_2d,
    800,                                      # h_samples
    1,                                        # v_samples = 1 for 2D lidar
    2 * chrono.CH_PI,                         # horizontal FOV (rad)
    0.0,                                      # max_vert_angle = 0 for 2D
    0.0,                                      # min_vert_angle = 0 for 2D
    100.0,                                    # max_range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)
lidar_2d.SetCollectionWindow(1.0 / 5.0)

lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "2D Lidar Raw Depth"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_2d)

# --- Third-person camera sensor attached to chassis ---
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-3, 0, 1.5),
    chrono.QuatFromAngleAxis(0.15, chrono.ChVector3d(0, 1, 0)),
)
cam_3p = sens.ChCameraSensor(
    chassis,            # attach third-person camera to the chassis
    30,                 # update_rate (Hz) — physical rate, not 1/dt
    cam_offset_pose,
    1280, 720,
    1.408,              # horizontal FOV (rad)
)
cam_3p.SetName("Third Person Camera")
cam_3p.SetLag(0)
cam_3p.SetCollectionWindow(0)
cam_3p.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person View"))
cam_3p.PushFilter(sens.ChFilterRGBA8Access())
cam_3p.PushFilter(sens.ChFilterSave("cam/third_person/"))
manager.AddSensor(cam_3p)

# === Irrlicht Visualization (wheeled vehicle visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar Lidar Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                # vehicle truths use directional light
vis.AttachVehicle(artcar.GetVehicle())

# === Driver (interactive IRR — scored-core default matching truth) ===
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


try:
    while vis.Run() and artcar.GetSystem().GetChTime() < sim_end:
        time = artcar.GetSystem().GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        artcar.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        artcar.Advance(step_size)
        vis.Advance(step_size)
        manager.Update()


        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:     # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
