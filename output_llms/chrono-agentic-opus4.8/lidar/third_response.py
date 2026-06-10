"""ARTcar wheeled vehicle on flat rigid terrain, instrumented with lidar sensors.

System type: NSC (rigid terrain catalog vehicle, Bullet collision).
Main bodies: the ARTcar wrapper (chassis + four wheels/tires/suspension), driven
by an interactive driver, rolling on a flat RigidTerrain patch.
Sensors attached to the chassis: a 3D lidar (multi-layer), a 2D lidar (single
layer), and a third-person RGB camera that rides behind/above the chassis.
Expected behavior: the car sits on the terrain at rest, the lidar sensors sweep
the surrounding scene each tick (depth + point-cloud streams), and the
third-person camera produces an RGB image stream of the car on the terrain.
"""

import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants === geometry / physics / sensor parameters (no bare literals downstream)
time_step = 1.0e-3                 # integration step
sim_end = 8.0                      # bounded recording run length (s)
render_fps = 50.0                  # Irrlicht review-frame cadence
render_step_size = 1.0 / render_fps

terrain_length = 100.0             # X size of the rigid patch
terrain_width = 100.0              # Y size of the rigid patch
terrain_top_z = 0.0                # flat patch top surface height

ARTCAR_REF_HEIGHT = 0.20           # chassis-origin height above wheel-bottom at rest
ARTCAR_TIRE_RADIUS = 0.10          # ARTcar tire radius (small 1/10-scale car)
init_x, init_y = 0.0, 0.0
init_z = terrain_top_z + ARTCAR_REF_HEIGHT          # precomputed once
init_loc = chrono.ChVector3d(init_x, init_y, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Lidar offset pose on the chassis (prompt: forward+up mount at (1.0, 0, 1)).
lidar_offset = chrono.ChVector3d(1.0, 0.0, 1.0)
lidar_update_rate = 5.0            # physical Hz
lidar_h_samples = 800             # horizontal beams
lidar_v_samples_3d = 300          # vertical layers (3D lidar)
lidar_max_range = 100.0
cam_update_rate = 30.0             # camera physical Hz


# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === ARTcar catalog wrapper owns its ChSystemNSC + chassis/wheels/joints
vehicle = veh.ARTcar()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)          # rigid-terrain tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.ARTcar wrapper) ===
sys = vehicle.GetSystem()                              # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for contact
chassis_body = vehicle.GetChassisBody()               # cache: main chassis rigid body, reused
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Footprint sanity: wheel bottoms must rest on (not through) the terrain.
veh_obj = vehicle.GetVehicle()
spindle_world = [veh_obj.GetSpindlePos(axle, side)
                 for axle in range(veh_obj.GetNumberAxles())
                 for side in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - ARTCAR_TIRE_RADIUS
assert wheel_bottom_z >= terrain_top_z - 0.05, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={terrain_top_z:.3f}; raise ARTCAR_REF_HEIGHT")

# === Terrain === flat rigid patch under the vehicle
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window (chase camera + sky + light)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar with Lidar Sensors")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 4.0, 0.4)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                              # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === interactive driver bound to the visual system (catalog-vehicle default)
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Sensors === ChSensorManager with a 3D lidar, a 2D lidar, and a chase camera
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)

lidar_pose = chrono.ChFramed(lidar_offset, chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))

# 3D lidar: multi-layer vertical fan over the surrounding scene.
lidar3d = sens.ChLidarSensor(
    chassis_body, lidar_update_rate, lidar_pose,
    lidar_h_samples, lidar_v_samples_3d,
    2 * chrono.CH_PI, chrono.CH_PI / 12, -chrono.CH_PI / 6, lidar_max_range,
    sens.LidarBeamShape_RECTANGULAR, 2, 0.003, 0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar3d.SetName("Lidar 3D")
lidar3d.SetLag(0)
lidar3d.SetCollectionWindow(1.0 / lidar_update_rate)
lidar3d.PushFilter(sens.ChFilterVisualize(lidar_h_samples, lidar_v_samples_3d, "Raw Lidar 3D Depth"))
lidar3d.PushFilter(sens.ChFilterDIAccess())
lidar3d.PushFilter(sens.ChFilterPCfromDepth())
lidar3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar 3D Point Cloud"))
lidar3d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar3d)

# 2D lidar: single horizontal layer (v_samples = 1, both vertical angles 0).
lidar2d = sens.ChLidarSensor(
    chassis_body, lidar_update_rate, lidar_pose,
    lidar_h_samples, 1,
    2 * chrono.CH_PI, 0.0, 0.0, lidar_max_range,
    sens.LidarBeamShape_RECTANGULAR, 2, 0.003, 0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar2d.SetName("Lidar 2D")
lidar2d.SetLag(0)
lidar2d.SetCollectionWindow(1.0 / lidar_update_rate)
lidar2d.PushFilter(sens.ChFilterVisualize(lidar_h_samples, 1, "Raw Lidar 2D Depth"))
lidar2d.PushFilter(sens.ChFilterDIAccess())
lidar2d.PushFilter(sens.ChFilterPCfromDepth())
lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar 2D Point Cloud"))
lidar2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar2d)

# Third-person RGB camera: rides behind & above the chassis, looking forward.
cam_pose = chrono.ChFramed(
    chrono.ChVector3d(-4.0, 0.0, 1.5),
    chrono.QuatFromAngleAxis(0.15, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(chassis_body, cam_update_rate, cam_pose, 1280, 720, 1.408)
cam.SetName("Third Person Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/third_person/"))
manager.AddSensor(cam)

# === Main loop === throttled rendering, full vehicle Synchronize/Advance stack
render_steps = max(1, round(render_step_size / time_step))   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()

os.makedirs("cam", exist_ok=True)   # guard against missing output dir

frame = 0
step_number = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        vis.Advance(time_step)
        manager.Update()                       # pump all sensors once per physics step


        step_number += 1
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:      # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
