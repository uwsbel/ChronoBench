"""MAN 5t vehicle on a rigid hilly grass terrain.

This self-contained PyChrono 9.0.0 script models a wrapper-managed MAN 5t
wheeled vehicle with NSC contact on a rigid height-map terrain. The truck starts
at (-20, 0, 1.5), renders with the vehicle Irrlicht visual system, and uses the
catalog interactive driver stack while the terrain supplies the hilly support.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Named parameters keep the vehicle, terrain, and loop configuration visible.
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = STEP_SIZE
RENDER_STEP_SIZE = 1.0 / 50.0
RENDER_STEPS = math.ceil(RENDER_STEP_SIZE / STEP_SIZE)  # precomputed once

INIT_LOC = chrono.ChVector3d(-20.0, 0.0, 1.5)
INIT_ROT = chrono.QUNIT
TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 40.0
TERRAIN_MIN_HEIGHT = -1.5
TERRAIN_MAX_HEIGHT = 1.5
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01


# === Data Paths ===
# Chrono and vehicle data roots locate bundled MAN, terrain, and texture assets.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


# === Vehicle ===
# The MAN wrapper owns the ChSystem; all terrain and visualization attach to it.
vehicle = veh.MAN_5t()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned system reused every step
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: chassis body for placement checks
vehicle_core = vehicle.GetVehicle()  # cache: vehicle handle for vis and mass
print("VEHICLE MASS: ", vehicle_core.GetMass())

# Wrapper-created essentials: system, chassis, axles, wheels, tires, driver,
# terrain, and vehicle-aware Irrlicht visualization are all advanced together.
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain ===
# A rigid height-map patch creates hills while retaining the requested rigid contact.
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)

height_patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    TERRAIN_MIN_HEIGHT,
    TERRAIN_MAX_HEIGHT,
)
height_patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 20.0, 20.0)
height_patch.SetColor(chrono.ChColor(0.6, 0.8, 0.45))
terrain.Initialize()

spawn_ground = terrain.GetHeight(INIT_LOC)  # cache: spawn support height
assert INIT_LOC.z > spawn_ground + 0.25, (
    f"vehicle start z={INIT_LOC.z:.3f} is too close to terrain height "
    f"{spawn_ground:.3f}; raise the initial chassis height"
)


# === Visualization ===
# Vehicle Irrlicht follows the chassis over the hilly terrain.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 5t on Hilly Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.25), 18.0, 2.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_core)


# === Driver ===
# The catalog interactive driver is the scored core; record mode supplies inputs.
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
driver.Initialize()


# === Main Loop ===
# Synchronize and advance every vehicle subsystem in the catalog order.
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run():
        time = system.GetChTime()


        if step_number % RENDER_STEPS == 0:
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
except (RuntimeError, ValueError, OSError) as exc:
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
