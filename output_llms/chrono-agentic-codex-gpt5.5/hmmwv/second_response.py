"""HMMWV path-following simulation on a 200 m rigid terrain.

The NSC vehicle model drives on a flat rigid terrain with Bullet collision, follows
a circular path, and shows the path, steering sentinel, and steering target with
sphere markers while using a constant 0.3 throttle command.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants: vehicle, terrain, and path dimensions ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 16.0
RENDER_FPS = 50.0
RENDER_STEP = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0
TERRAIN_THICKNESS = 1.0
TERRAIN_Z = -0.5

PATH_RADIUS = 30.0
PATH_RUN = 10.0
PATH_TURNS = 2
PATH_START = chrono.ChVector3d(-40.0, -30.0, 0.5)
PATH_CENTER = chrono.ChVector3d(-30.0, 0.0, 0.06)
INIT_POS = PATH_START
INIT_ROT = chrono.QUNIT
CONSTANT_THROTTLE = 0.3
TARGET_SPEED = 8.0


# === Small visual marker helper ===
def make_sphere_marker(system, name, radius, color, pos):
    marker = chrono.ChBody()
    marker.SetName(name)
    marker.SetFixed(True)
    marker.SetPos(pos)
    shape = chrono.ChVisualShapeSphere(radius)
    shape.SetColor(color)
    marker.AddVisualShape(shape, chrono.ChFramed())
    system.AddBody(marker)
    return marker


# === Vehicle: HMMWV wrapper owns the Chrono system ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned system reused by terrain and markers
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = hmmwv.GetVehicle()  # cache: wrapper vehicle reused by driver and visualization
chassis = hmmwv.GetChassisBody()  # cache: chassis body available for diagnostics and cameras

print("VEHICLE MASS: ", vehicle.GetMass())

# Wrapper-created components are present: system, chassis, powertrain, axles,
# steering, tires, terrain contact, visualization, and autonomous driver.
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain: enlarged rigid patch for the circular path ===
terrain = veh.RigidTerrain(system)
terrain_mat = chrono.ChContactMaterialNSC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)

terrain_frame = chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, TERRAIN_Z), chrono.QUNIT)
patch = terrain.AddPatch(
    terrain_mat,
    terrain_frame,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    TERRAIN_THICKNESS,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.7, 0.75, 0.65))
terrain.Initialize()


# === Path and controller: circular path with PID steering ===
path = veh.CirclePath(PATH_START, PATH_RADIUS, PATH_RUN, True, PATH_TURNS)
driver = veh.ChPathFollowerDriver(vehicle, path, "circular_path", TARGET_SPEED)
steering_controller = driver.GetSteeringController()  # cache: queried every step for markers
speed_controller = driver.GetSpeedController()  # cache: configured once
steering_controller.SetLookAheadDistance(5.0)
steering_controller.SetGains(0.8, 0.0, 0.05)
speed_controller.SetGains(0.4, 0.0, 0.0)
driver.Initialize()


# === Visualization markers: path balls plus live controller points ===
make_sphere_marker(
    system,
    "path_start_ball",
    0.35,
    chrono.ChColor(0.0, 0.4, 1.0),
    chrono.ChVector3d(PATH_START.x, PATH_START.y, 0.08),
)
make_sphere_marker(
    system,
    "path_circle_ball",
    0.35,
    chrono.ChColor(0.0, 0.9, 0.2),
    PATH_CENTER,
)
sentinel_marker = make_sphere_marker(
    system,
    "steering_sentinel",
    0.22,
    chrono.ChColor(1.0, 0.8, 0.0),
    chrono.ChVector3d(PATH_START.x, PATH_START.y, 0.35),
)
target_marker = make_sphere_marker(
    system,
    "steering_target",
    0.22,
    chrono.ChColor(1.0, 0.0, 0.0),
    chrono.ChVector3d(PATH_START.x + 3.0, PATH_START.y, 0.35),
)


# === Irrlicht visualization: vehicle-specific visual system ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Circular Path Follower")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


# === Review-only output setup ===


# === Main loop: synchronize and advance the complete vehicle stack ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver.Synchronize(time)
        driver.SetThrottle(CONSTANT_THROTTLE)
        driver.SetBraking(0.0)
        driver_inputs = driver.GetInputs()

        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        sentinel = steering_controller.GetSentinelLocation()
        target = steering_controller.GetTargetLocation()
        sentinel_marker.SetPos(chrono.ChVector3d(sentinel.x, sentinel.y, 0.35))
        target_marker.SetPos(chrono.ChVector3d(target.x, target.y, 0.35))


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError, OSError) as exc:
    print(f"Simulation failed with a named runtime or I/O error: {exc}")
    raise
finally:
    pass


# === Review-only video and plot assembly ===
