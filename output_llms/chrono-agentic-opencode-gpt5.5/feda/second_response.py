"""FEDA double-lane-change maneuver on enlarged rigid terrain.

This standalone PyChrono 9.0.x simulation uses the FED-Alpha catalog vehicle with
an NSC rigid-terrain contact model.  The vehicle starts at (-50, 0, 0.5), follows
an ISO-style double lane change through a path-follower cruise driver, and targets
10 m/s over a 200 m terrain patch.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named setup values make the requested maneuver explicit
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

INIT_LOC = chrono.ChVector3d(-50.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
TARGET_SPEED = 10.0

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 20.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

PATH_LENGTH = 13.5
PATH_WIDTH = 4.0
PATH_OFFSET = 11.0
PATH_TOTAL_LENGTH = 50.0
PATH_TURNS_LEFT = True


# === Vehicle and system === the FEDA wrapper owns the ChSystemNSC and body stack
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.FEDA()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
vehicle.SetTireType(veh.TireModelType_PAC02)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned system reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: chassis body used for camera and logging
veh_core = vehicle.GetVehicle()  # cache: vehicle core reused for mass, path driver, wheels
print("VEHICLE MASS: ", veh_core.GetMass())

# Wrapper-created essentials: chassis, suspensions, steering links, wheels, tires,
# driveline, powertrain, and joints are created inside veh.FEDA.Initialize().
spindle_positions = []
for axle_index in range(veh_core.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_core.GetSpindlePos(axle_index, side))
assert min(p.z for p in spindle_positions) > 0.15, "FEDA wheel spindles must start above the terrain plane"

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === a 200 m rigid patch gives the path room to fit on the road
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Driver === autonomous path follower with requested target speed and gains
path = veh.DoubleLaneChangePath(
    INIT_LOC,
    PATH_LENGTH,
    PATH_WIDTH,
    PATH_OFFSET,
    PATH_TOTAL_LENGTH,
    PATH_TURNS_LEFT,
)
driver = veh.ChPathFollowerDriver(veh_core, path, "iso_double_lane_change", TARGET_SPEED)
steering_controller = driver.GetSteeringController()  # cache: controller configured once
speed_controller = driver.GetSpeedController()  # cache: controller configured once
steering_controller.SetLookAheadDistance(5.0)
steering_controller.SetGains(0.8, 0.0, 0.0)
speed_controller.SetGains(0.4, 0.0, 0.0)
driver.Initialize()


# === Visualization === vehicle-aware Irrlicht window, initialized before scene elements
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA ISO Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_core)


# === Main loop === synchronize driver, terrain, vehicle, and visual system each step
realtime_timer = chrono.ChRealtimeStepTimer()


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

            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid state during stepping
    traceback.print_exc()
    raise
finally:
    pass
