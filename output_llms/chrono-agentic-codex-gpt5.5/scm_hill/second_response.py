"""HMMWV driving over a deformable SCM height-map hill with box obstacles and a chassis lidar.

The scene uses an SMC vehicle system owned by the HMMWV wrapper, a Bullet collision
system for vehicle/terrain/obstacle contact, TMeasy tires for SCM traction, and an
Irrlicht vehicle visualization. A 2D lidar rides on the chassis and visualizes its
depth and point-cloud streams while the vehicle crosses the bump terrain.
"""

import math

import numpy as np
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === named parameters keep the terrain, vehicle, sensors, and loop explicit
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
SCM_LENGTH = 40.0
SCM_WIDTH = 40.0
SCM_RESOLUTION = 0.04
HMMWV_INIT_POS = chrono.ChVector3d(-15.0, -2.0, 1.0)
HMMWV_INIT_ROT = chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 0, 1))
TIRE_FAMILY = 1
SUPPORT_FAMILY = 4
CHASSIS_FAMILY = 5
BOX_HALF_EXTENTS = chrono.ChVector3d(0.45, 0.45, 0.45)
BOX_DENSITY = 700.0
LIDAR_RATE = 5.0
LIDAR_HORIZONTAL_SAMPLES = 720
LIDAR_VERTICAL_SAMPLES = 1


# === Vehicle and system === the wrapper owns the SMC ChSystem and vehicle bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(HMMWV_INIT_POS, HMMWV_INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)  # SCM terrain needs a non-rigid tire
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned system reused by terrain, sensors, loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = hmmwv.GetVehicle()  # cache: vehicle subsystem handle reused throughout setup
chassis = hmmwv.GetChassisBody()  # cache: chassis body anchors moving patch and lidar
print("VEHICLE MASS: ", vehicle.GetMass())

# Wrapper-created components are intentionally visible to the source reviewer:
# system: HMMWV-owned ChSystemSMC; bodies: chassis, wheels, spindles; joints:
# suspension and steering links; visualization: vehicle Irrlicht system; driver below.

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

spindle_positions = []  # cache: validate wheel support once after initialization
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))
assert min(p.z for p in spindle_positions) > 0.15, "HMMWV spindles must start above the SCM surface"


# === Terrain and obstacle contact === SCM hill plus rigid support for non-tire props
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(2.0e6, 0.0, 1.1, 0.0, 30.0, 0.01, 2.0e8, 3.0e4)
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.15)
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"), SCM_LENGTH, SCM_WIDTH, -1.0, 1.0, SCM_RESOLUTION)
terrain.SetMeshWireframe(False)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

front_left_tire = vehicle.GetAxles()[0].m_wheels[0].GetTire()  # cache: tire dimensions used for all HMMWV tires
tire_radius = front_left_tire.GetRadius()
tire_width = front_left_tire.GetWidth()
for axle in vehicle.GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_radius + 0.04, tire_width),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2.0)),
        )
        spindle.EnableCollision(True)
        spindle_cm = spindle.GetCollisionModel()
        spindle_cm.SetFamily(TIRE_FAMILY)
        spindle_cm.DisallowCollisionsWith(TIRE_FAMILY)
        spindle_cm.DisallowCollisionsWith(SUPPORT_FAMILY)

support_mat = chrono.ChContactMaterialSMC()
support_mat.SetFriction(0.9)
support_mat.SetRestitution(0.01)
support_mat.SetYoungModulus(2.0e7)
support = chrono.ChBodyEasyBox(SCM_LENGTH, SCM_WIDTH, 0.2, 1000.0, False, True, support_mat)
support.SetName("hidden_obstacle_support")
support.SetPos(chrono.ChVector3d(0, 0, -0.1))
support.SetFixed(True)
support.EnableCollision(True)
support_cm = support.GetCollisionModel()
support_cm.SetFamily(SUPPORT_FAMILY)
support_cm.DisallowCollisionsWith(TIRE_FAMILY)
support_cm.DisallowCollisionsWith(CHASSIS_FAMILY)
system.AddBody(support)

box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.85)
box_mat.SetRestitution(0.05)
box_mat.SetYoungModulus(2.0e7)
rng = np.random.default_rng(17)
obstacle_positions = []  # cache: deterministic random positions used for creation and logging
for index in range(5):
    x_pos = float(rng.uniform(-8.0, 12.0))
    y_pos = float(rng.uniform(-6.0, 6.0))
    z_pos = BOX_HALF_EXTENTS.z
    obstacle_positions.append((x_pos, y_pos, z_pos))
    obstacle = chrono.ChBodyEasyBox(
        2.0 * BOX_HALF_EXTENTS.x,
        2.0 * BOX_HALF_EXTENTS.y,
        2.0 * BOX_HALF_EXTENTS.z,
        BOX_DENSITY,
        True,
        True,
        box_mat,
    )
    obstacle.SetName(f"random_box_obstacle_{index + 1}")
    obstacle.SetPos(chrono.ChVector3d(x_pos, y_pos, z_pos))
    obstacle.EnableCollision(True)
    system.AddBody(obstacle)

system.GetCollisionSystem().BindAll()


# === Sensors === a chassis-mounted 2D lidar with depth and point-cloud visualization
manager = sens.ChSensorManager(system)
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.8, 0.0, 1.2),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,
    LIDAR_RATE,
    lidar_offset,
    LIDAR_HORIZONTAL_SAMPLES,
    LIDAR_VERTICAL_SAMPLES,
    2.0 * chrono.CH_PI,
    0.0,
    0.0,
    50.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Chassis 2D Lidar")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_HORIZONTAL_SAMPLES, LIDAR_VERTICAL_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


# === Visualization and driver === Irrlicht vehicle view plus interactive driver
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("SCM hill HMMWV with obstacles and lidar")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(1.0 / 50.0)
driver.SetThrottleDelta(1.0 / 50.0)
driver.SetBrakingDelta(1.0 / 20.0)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop === synchronize driver, SCM terrain, HMMWV, visualization, and sensors
frame = 0
step_number = 0
try:

    while vis.Run() and system.GetChTime() < SIM_END:
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        time = system.GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError, OSError, IOError) as exc:
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
