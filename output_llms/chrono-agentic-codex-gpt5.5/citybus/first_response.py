"""CityBus rigid-terrain simulation using PyChrono NSC contact and Irrlicht.

The model builds a catalog CityBus with mixed mesh and primitive visualization,
places it on a flat textured RigidTerrain patch, and exposes an interactive
keyboard driver for steering, throttle, and braking. The vehicle visual system
follows the bus from a chase-camera offset while the subsystem stack advances at
50 rendered frames per second.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Named parameters keep the vehicle setup and loop cadence visible.
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_TEXTURE = "terrain/textures/concrete.jpg"

INIT_LOCATION = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROTATION = chrono.QUNIT
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 2.5)
CHASE_DISTANCE = 18.0
CHASE_HEIGHT = 2.5


# === Vehicle ===
# Catalog data paths and wrapper-owned system match Chrono vehicle demos.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(INIT_LOCATION, INIT_ROTATION))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(TIRE_STEP_SIZE)
bus.Initialize()

system = bus.GetSystem()  # cache: wrapper-owned ChSystem reused below
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = bus.GetVehicle()  # cache: underlying wheeled vehicle reused by vis/driver
chassis = bus.GetChassisBody()  # cache: chassis state logged during validation
print("VEHICLE MASS: ", vehicle.GetMass())

# Wrapper-created essentials: system, chassis body, axles/wheels/tires, terrain,
# vehicle-aware Irrlicht visualizer, and interactive driver all share one system.
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain ===
# A flat RigidTerrain patch provides the textured support surface for the bus.
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile(TERRAIN_TEXTURE), 200.0, 200.0)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Visualization And Driver ===
# Vehicle-aware Irrlicht visualization owns the chase camera and keyboard driver.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
driver.Initialize()


# === Main Loop ===
# Synchronize the full vehicle stack and render at the requested 50 FPS cadence.
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame += 1

        for _ in range(RENDER_STEPS):
            time = system.GetChTime()
            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()

            terrain.Synchronize(time)
            bus.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            bus.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)


            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(RENDER_STEP_SIZE)
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid numeric state
    traceback.print_exc()
    raise
finally:
    pass


# === Review Output ===
