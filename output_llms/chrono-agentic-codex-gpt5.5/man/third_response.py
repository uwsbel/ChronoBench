"""MAN 10t vehicle scene with rigid grass terrain, scattered boxes, and lidar.

The simulation uses a wrapper-managed NSC vehicle system. A MAN_10t truck drives
on a rigid terrain patch textured with grass while deterministic random boxes
rest on the ground ahead of the vehicle. A chassis-mounted lidar is managed by
pychrono.sensor and updated during the vehicle loop.
"""

import math
import numpy as np

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Constants ===
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 50.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOC = chrono.ChVector3d(-20.0, 0.0, 0.8)
INIT_ROT = chrono.QuatFromAngleZ(0.0)
BOX_COUNT = 12
BOX_DENSITY = 600.0
BOX_SEED = 43


# === Vehicle and terrain ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.MAN_10t()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()
system = vehicle.GetSystem()  # cache: wrapper-owned system reused by terrain, sensors, and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

chassis = vehicle.GetChassisBody()  # cache: lidar parent and optional contact proxy
veh_model = vehicle.GetVehicle()  # cache: repeated diagnostic handle

# Wrapper-created components are visible here: vehicle owns the system, chassis,
# axles, tires, driver, terrain, and visual system; all share this one system.
terrain_mat = chrono.ChContactMaterialNSC()
terrain_mat.SetFriction(TERRAIN_FRICTION)
terrain_mat.SetRestitution(TERRAIN_RESTITUTION)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    terrain_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 80, 80)
patch.SetColor(chrono.ChColor(0.45, 0.65, 0.32))
terrain.Initialize()

spindle_z = []
for axle_index in range(veh_model.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_z.append(veh_model.GetSpindlePos(axle_index, side).z)
assert min(spindle_z) > 0.1, "MAN_10t wheel centers must initialize above the rigid terrain"


# === Random boxes ===
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.75)
box_mat.SetRestitution(0.05)
rng = np.random.default_rng(BOX_SEED)
boxes = []
for i in range(BOX_COUNT):
    sx = float(rng.uniform(0.7, 1.6))
    sy = float(rng.uniform(0.7, 1.8))
    sz = float(rng.uniform(0.5, 1.4))
    x = float(rng.uniform(-14.0, 18.0))
    y = float(rng.uniform(-8.0, 8.0))
    box = chrono.ChBodyEasyBox(sx, sy, sz, BOX_DENSITY, True, True, box_mat)
    box.SetName(f"random_box_{i:02d}")
    box.SetPos(chrono.ChVector3d(x, y, 0.5 * sz))
    box.SetRot(chrono.QuatFromAngleZ(float(rng.uniform(-math.pi, math.pi))))
    box.SetFixed(True)
    box.EnableCollision(True)
    system.AddBody(box)
    boxes.append(box)

system.GetCollisionSystem().BindAll()


# === Sensor manager and lidar ===
manager = sens.ChSensorManager(system)
lidar_update_rate = 5.0
horizontal_samples = 720
vertical_samples = 1
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0.0, 2.4),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,
    lidar_update_rate,
    lidar_offset,
    horizontal_samples,
    vertical_samples,
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
lidar.SetName("MAN 10t 2D Lidar")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


# === Visualization and driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 10t lidar over grass terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_EVERY * STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_EVERY * STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_EVERY * STEP_SIZE / 0.3)
driver.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0

try:

    while vis.Run() and system.GetChTime() < SIM_END:
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frame += 1

        driver_inputs = driver.GetInputs()
        time = system.GetChTime()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError, OSError) as exc:
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
