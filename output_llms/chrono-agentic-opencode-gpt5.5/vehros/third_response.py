"""HMMWV vehicle ROS/lidar demo using an NSC rigid-terrain system.

The simulation builds a catalog HMMWV on a flat rigid terrain patch, adds a
fixed box target for lidar returns, attaches a lidar sensor to the chassis, and
publishes simulation clock, chassis state, TF, driver input, and lidar point
cloud data through Chrono ROS2 handlers.  The vehicle starts from rest and can
receive driver commands through ROS while the Irrlicht window shows the scene.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.ros as chros


# === Constants ===
step_size = 5.0e-3
tire_step_size = step_size
sim_end = 2.0
render_fps = 15.0
render_every = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once
init_loc = chrono.ChVector3d(0.0, 0.0, 0.6)
init_rot = chrono.QUNIT
terrain_length = 80.0
terrain_width = 20.0
box_pos = chrono.ChVector3d(12.0, 1.5, 0.75)
box_size = chrono.ChVector3d(1.0, 1.0, 1.5)
lidar_update_rate = 5.0
lidar_h_samples = 160
lidar_v_samples = 8
ros_update_enabled = True


# === Vehicle and terrain ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemNSC reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

chassis = hmmwv.GetChassisBody()  # cache: chassis body reused by lidar and ROS handlers
chassis.SetName("hmmwv_chassis")
vehicle_model = hmmwv.GetVehicle()  # cache: wrapper vehicle reused in vis, ROS, and loop
# wheels/spindles, suspension, steering, driveline, and joints are created by the HMMWV wrapper.

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 80, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Scene bodies ===
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.01)
target_box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 1000.0, True, True, box_mat)
target_box.SetName("lidar_visualization_box")
target_box.SetPos(box_pos)
target_box.SetFixed(True)
target_box.EnableCollision(True)
system.AddBody(target_box)

base_link = chrono.ChBody()
base_link.SetName("base_link")
base_link.SetFixed(True)
system.AddBody(base_link)


# === Sensors and ROS ===
sensor_manager = sens.ChSensorManager(system)

lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-0.5, 0.0, 1.6),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
lidar = sens.ChLidarSensor(
    chassis,
    lidar_update_rate,
    lidar_offset,
    lidar_h_samples,
    lidar_v_samples,
    2.0 * chrono.CH_PI,
    chrono.CH_PI / 12.0,
    -chrono.CH_PI / 12.0,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("HMMWV Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)
lidar.PushFilter(sens.ChFilterVisualize(lidar_h_samples, lidar_v_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
sensor_manager.AddSensor(lidar)


# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV ROS Lidar")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(-5.0, 2.5, 1.5), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_model)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / render_fps) / 1.0)
driver.SetThrottleDelta((1.0 / render_fps) / 1.0)
driver.SetBrakingDelta((1.0 / render_fps) / 0.3)
driver.Initialize()

ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25.0, chassis, "~/output/hmmwv_chassis"))
tf_handler = chros.ChROSTFHandler(25.0)
tf_handler.AddTransform(base_link, "base_link", chassis, "hmmwv_chassis")
tf_handler.AddSensor(lidar, "hmmwv_chassis", "hmmwv_lidar")
ros_manager.RegisterHandler(tf_handler)
ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25.0, driver, "~/input/driver_inputs"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar_update_rate, lidar, "~/output/lidar/data"))
ros_manager.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    ros_ok = True
    step_count = 0
    max_steps = math.ceil(sim_end / step_size)  # precomputed once
    while vis.Run() and system.GetChTime() < sim_end and ros_ok and step_count < max_steps:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()  # cache: used by vehicle and visual synchronization

            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(step_size)
            terrain.Advance(step_size)
            hmmwv.Advance(step_size)
            vis.Advance(step_size)
            sensor_manager.Update()

            if ros_update_enabled and not ros_manager.Update(time, step_size):
                ros_ok = False
                break


            realtime_timer.Spin(step_size)
            step_count += 1
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state guard
    print(f"Chrono runtime failure: {exc}")
    raise
except (OSError, IOError) as exc:  # file, display, or ROS transport resource guard
    print(f"Resource failure during simulation: {exc}")
    raise
finally:
    pass
