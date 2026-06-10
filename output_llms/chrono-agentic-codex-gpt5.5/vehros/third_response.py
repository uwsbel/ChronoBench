"""HMMWV rigid-terrain ROS simulation with lidar.

This PyChrono 9.0.0 script models a full HMMWV vehicle on rigid terrain using
the wrapper-owned NSC vehicle system.  The vehicle chassis publishes body state,
driver inputs are exposed through ROS, and a chassis-mounted lidar is managed by
the Chrono sensor manager and published through a ROS lidar handler.  The scene
uses Irrlicht visualization with a camera at (-5, 2.5, 1.5).
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as chros


# === Constants ===
step_size = 1.0e-3
tire_step_size = step_size
render_step_size = 1.0 / 50.0
render_steps = max(1, math.ceil(render_step_size / step_size))
sim_end = 8.0
terrain_length = 200.0
terrain_width = 100.0
terrain_height = 0.0
init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.QUNIT
lidar_update_rate = 5.0
lidar_horizontal_samples = 240
lidar_vertical_samples = 1


# === Vehicle wrapper, system, and terrain ===
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

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = hmmwv.GetChassisBody()  # cache: main vehicle rigid body owned by HMMWV_Full
chassis.SetName("hmmwv_chassis")
vehicle = hmmwv.GetVehicle()  # cache: wrapper-created ChWheeledVehicle
# Wrapper-created components are the HMMWV chassis, suspension links, steering
# joints, wheel bodies, tire subsystems, engine, driveline, and brake models.

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, terrain_height), chrono.QUNIT),
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.05)
visualization_box = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000.0, True, True, box_mat)
visualization_box.SetName("visualization_box")
visualization_box.SetPos(chrono.ChVector3d(8.0, 1.5, 0.45))
visualization_box.SetFixed(True)
box_vis_mat = chrono.ChVisualMaterial()
box_vis_mat.SetDiffuseColor(chrono.ChColor(0.1, 0.4, 0.9))
visualization_box.GetVisualShape(0).SetMaterial(0, box_vis_mat)
system.Add(visualization_box)

print("VEHICLE MASS:", vehicle.GetMass())


# === Sensor manager and lidar ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(0.0, 0.0, 12.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)
lidar_pose = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 1.4),
    chrono.QUNIT,
)
lidar = sens.ChLidarSensor(
    chassis,
    lidar_update_rate,
    lidar_pose,
    lidar_horizontal_samples,
    lidar_vertical_samples,
    2.0 * chrono.CH_PI,
    0.0,
    0.0,
    80.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("hmmwv_lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


# === Irrlicht vehicle visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV ROS Lidar")
vis.SetWindowSize(1280, 720)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddCamera(chrono.ChVector3d(-5.0, 2.5, 1.5), chrono.ChVector3d(0.0, 0.0, 0.8))
vis.AttachVehicle(vehicle)


# === Driver and ROS handlers ===
driver = veh.ChDriver(vehicle)
driver.Initialize()

ros_manager = chros.ChROSPythonManager("chrono_hmmwv_lidar")
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25.0, chassis, "~/output/hmmwv_chassis"))
ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25.0, driver, "~/input/driver_inputs"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar_update_rate, lidar, "~/output/lidar"))
tf_handler = chros.ChROSTFHandler(25.0)
tf_handler.AddSensor(lidar, "hmmwv_chassis", "hmmwv_lidar")
ros_manager.RegisterHandler(tf_handler)
ros_manager.Initialize()


# === Simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.SetCameraPosition(chrono.ChVector3d(-5.0, 2.5, 1.5))
            vis.SetCameraTarget(chrono.ChVector3d(0.0, 0.0, 0.8))
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)
        manager.Update()

        if not ros_manager.Update(time, step_size):
            break


        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:
    print(f"Simulation runtime/state failure: {exc}")
    raise
except (OSError, IOError) as exc:
    print(f"Simulation file I/O failure: {exc}")
    raise
