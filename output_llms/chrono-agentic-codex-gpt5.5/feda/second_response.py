"""FEDA double lane change maneuver on a rigid NSC terrain patch.

The script builds a FED-Alpha vehicle, places it at (-50, 0, 0.5), and drives it
with a path-follower/cruise-control driver over a 200 m terrain patch.  The
expected behavior is a stable ISO-style double lane change at a 10 m/s target
speed.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Named parameters keep the requested maneuver dimensions and control settings visible.
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = 1e-3
SIM_END = 15.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 40.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOC = chrono.ChVector3d(-50.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
PATH_START = chrono.ChVector3d(-50.0, 0.0, 0.5)
PATH_LENGTH = 13.5
PATH_WIDTH = 4.0
PATH_OFFSET = 11.0
PATH_TOTAL_LENGTH = 100.0
TARGET_SPEED = 10.0
LOOK_AHEAD_DISTANCE = 5.0
STEERING_GAINS = (0.8, 0.0, 0.0)
SPEED_GAINS = (0.4, 0.0, 0.0)


# === Vehicle & Terrain ===
# FEDA owns the Chrono system; terrain and visualization attach to that system.
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

system = vehicle.GetSystem()  # cache: wrapper-owned system reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
feda_vehicle = vehicle.GetVehicle()  # cache: Chrono vehicle handle reused below
chassis = vehicle.GetChassisBody()  # cache: chassis body reused for diagnostics
print("VEHICLE MASS: ", feda_vehicle.GetMass())

# Wrapper-created essentials: system, chassis/body tree, suspension, wheels, tires,
# powertrain, driver, terrain, and vehicle-aware Irrlicht visualization.
spindle_positions = []  # cache: measured once immediately after initialization
tire_radius = feda_vehicle.GetAxle(0).m_wheels[veh.LEFT].GetTire().GetRadius()
for axle_index in range(feda_vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(feda_vehicle.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - tire_radius
assert wheel_bottom_z >= -0.05, (
    f"vehicle wheel bottom is below terrain by {-wheel_bottom_z:.3f} m"
)

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 40)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Path-Follower Driver ===
# The driver replaces manual input with a double-lane-change path and cruise control.
path = veh.DoubleLaneChangePath(
    PATH_START,
    PATH_LENGTH,
    PATH_WIDTH,
    PATH_OFFSET,
    PATH_TOTAL_LENGTH,
    True,
)
driver = veh.ChPathFollowerDriver(feda_vehicle, path, "iso_double_lane_change", TARGET_SPEED)
steering_controller = driver.GetSteeringController()  # cache: configured once
speed_controller = driver.GetSpeedController()  # cache: configured once
steering_controller.SetLookAheadDistance(LOOK_AHEAD_DISTANCE)
steering_controller.SetGains(*STEERING_GAINS)
speed_controller.SetGains(*SPEED_GAINS)
driver.Initialize()

target_marker = chrono.ChBody()
target_marker.SetFixed(True)
target_shape = chrono.ChVisualShapeSphere(0.25)
target_shape.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
target_marker.AddVisualShape(target_shape, chrono.ChFramed())
system.AddBody(target_marker)


# === Visualization ===
# Vehicle-aware Irrlicht renders the FEDA model, terrain, and driver HUD.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA ISO Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(feda_vehicle)


# === Review Logging ===


# === Main Loop ===
# Synchronize and advance every vehicle subsystem once per physics step.
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            target_marker.SetPos(steering_controller.GetTargetLocation())
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:  # Chrono solver or invalid-state failure
    traceback.print_exc()
    raise
finally:
    pass


# === Review Artifacts ===
