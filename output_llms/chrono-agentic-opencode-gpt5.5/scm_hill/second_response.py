"""HMMWV driving over a deformable SCM heightmap hill with five box obstacles and a chassis-mounted lidar.

The model uses the SMC vehicle/contact formulation with Bullet collision, TMeasy tires,
SCM Bekker-Wong soil, fixed obstacle boxes, and an OptiX lidar managed by
ChSensorManager. The expected behavior is that the vehicle moves across the bumpy
soft terrain while the lidar point-cloud visualization updates with the scene.
"""

import math
import traceback

import numpy as np
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === simulation timing, terrain, vehicle, and lidar parameters
STEP_SIZE = 0.002
TIRE_STEP_SIZE = 0.001
SIM_END = 8.0
RENDER_FPS = 25.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
INIT_POS = chrono.ChVector3d(-14.0, 0.0, 1.0)
INIT_ROT = chrono.QUNIT
TERRAIN_LENGTH = 40.0
TERRAIN_WIDTH = 40.0
TERRAIN_RESOLUTION = 0.05


# === Vehicle === wrapper-created HMMWV owns the SMC system and all vehicle bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemSMC reused throughout
chassis = vehicle.GetChassisBody()  # cache: chassis used by SCM moving patch and sensors
veh_obj = vehicle.GetVehicle()  # cache: vehicle interface used for wheel checks
# bodies: chassis, suspension, steering, wheels, and tire bodies are created by HMMWV_Full
# joints: suspension and steering constraints are created inside the vehicle wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === SCM heightmap hill with active domain around the chassis
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(2e6, 0.0, 1.1, 0.0, 30.0, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetMeshWireframe(False)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"), TERRAIN_LENGTH, TERRAIN_WIDTH, -1.0, 1.0, TERRAIN_RESOLUTION)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

spindle_positions = []
for axle_index in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_obj.GetSpindlePos(axle_index, side))
tire_radius = veh_obj.GetAxle(0).m_wheels[0].GetTire().GetRadius()  # cache: radius for spawn check
terrain_height = terrain.GetHeight(chrono.ChVector3d(INIT_POS.x, INIT_POS.y, 0.0))
wheel_bottom_z = min(p.z for p in spindle_positions) - tire_radius
assert wheel_bottom_z >= terrain_height - 0.10, (
    f"vehicle sinks into SCM at spawn: wheel bottom z={wheel_bottom_z:.3f}, "
    f"terrain height={terrain_height:.3f}"
)

TIRE_FAMILY = 1
SUPPORT_FAMILY = 4
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
first_tire = veh_obj.GetAxle(0).m_wheels[0].GetTire()  # cache: geometry source for tire cylinders
tire_rad = first_tire.GetRadius()
tire_width = first_tire.GetWidth()
for axle in veh_obj.GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_width),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        spindle_cm = spindle.GetCollisionModel()
        spindle_cm.SetFamily(TIRE_FAMILY)
        spindle_cm.DisallowCollisionsWith(TIRE_FAMILY)
        spindle_cm.DisallowCollisionsWith(SUPPORT_FAMILY)
system.GetCollisionSystem().BindAll()


# === Obstacles === five deterministic fixed box obstacles for lidar and vehicle avoidance
obstacle_mat = chrono.ChContactMaterialSMC()
obstacle_mat.SetFriction(0.8)
obstacle_mat.SetRestitution(0.02)
obstacle_mat.SetYoungModulus(2e7)
rng = np.random.default_rng(2)
obstacles = []
for i in range(5):
    x = float(rng.uniform(-4.0, 13.0))
    y = float(rng.uniform(-7.0, 7.0))
    if abs(y) < 1.4:
        y += 2.5 if y >= 0 else -2.5
    box = chrono.ChBodyEasyBox(1.0, 1.0, 0.8, 1000.0, True, True, obstacle_mat)
    box.SetName(f"obstacle_box_{i + 1}")
    z = terrain.GetHeight(chrono.ChVector3d(x, y, 0.0)) + 0.4
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetFixed(True)
    box.EnableCollision(True)
    system.AddBody(box)
    obstacles.append(box)


# === Sensors === lidar is attached to the vehicle chassis and updated by ChSensorManager
manager = sens.ChSensorManager(system)
lidar_update_rate = 5.0
horizontal_samples = 800
vertical_samples = 32
lidar = sens.ChLidarSensor(
    chassis,
    lidar_update_rate,
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 1.6), chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0))),
    horizontal_samples,
    vertical_samples,
    2.0 * chrono.CH_PI,
    chrono.CH_PI / 12.0,
    -chrono.CH_PI / 12.0,
    60.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


# === Visualization === vehicle-aware Irrlicht window with sky, logo, and directional light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("SCM Hill HMMWV with Lidar")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())


# === Driver === interactive driver for the vehicle demo loop
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop === render cadence loop with full vehicle, terrain, sensor synchronization
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            driver.Synchronize(time)
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            vehicle.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            manager.Update()

            if system.GetChTime() >= SIM_END:
                break
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
