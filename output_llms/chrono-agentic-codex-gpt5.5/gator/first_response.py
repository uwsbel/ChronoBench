"""Gator vehicle driving on a flat rigid terrain with an NSC contact system.

The script builds a self-contained PyChrono 9.0 vehicle scene: a Gator wrapper,
TMEASY tires, mesh visualization, textured RigidTerrain, an Irrlicht vehicle
visualizer, and an interactive IRR driver for steering, throttle, and braking.
The expected behavior is a real-time vehicle demo running at a 50 Hz render
cadence while the vehicle dynamics advance with a smaller tire/vehicle step.
"""

import csv
import math
import os
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Parameters === named constants keep geometry, vehicle setup, and timing explicit
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = 1e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_STEP = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP / STEP_SIZE))

TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TEXTURE_U_TILES = 80.0
TEXTURE_V_TILES = 80.0

INIT_LOCATION = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROTATION = chrono.QUNIT
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3
REVIEW_THROTTLE = 0.55
REVIEW_STEERING_AMPLITUDE = 0.12


# === Vehicle setup === catalog wrapper owns the system and all vehicle bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOCATION, INIT_ROTATION))
vehicle.SetTireType(veh.TireModelType_TMEASY)  # prompt: TMEASY tire model
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemNSC reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: main chassis body reused for checks/logging
veh_core = vehicle.GetVehicle()  # cache: wrapper-created wheeled vehicle aggregate
# wheels/spindles, suspension links, steering links, drivetrain, and brakes are
# created by the veh.Gator wrapper; terrain, visualization, and driver are below.

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === flat rigid support with an NSC material and custom texture
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
patch.SetTexture(
    veh.GetDataFile("terrain/textures/tile4.jpg"),
    TEXTURE_U_TILES,
    TEXTURE_V_TILES,
)
patch.SetColor(chrono.ChColor(0.82, 0.82, 0.64))
terrain.Initialize()


# === Spawn verification === validate the Gator starts on the rigid terrain
tire_radius = 0.0
try:
    tire_radius = veh_core.GetAxles()[0].m_wheels[0].GetTire().GetRadius()
except (AttributeError, IndexError, RuntimeError) as exc:
    print(f"tire radius lookup failed, using conservative fallback: {exc}")
    tire_radius = 0.28

spindle_positions = []
for axle_index in range(veh_core.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_core.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - tire_radius
assert wheel_bottom_z >= -0.08, (
    f"vehicle starts too low for the terrain: wheel bottom z={wheel_bottom_z:.3f}"
)


# === Visualization === vehicle-specific Irrlicht window with sky, logo, and light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())


# === Driver === interactive IRR controls steering, throttle, and braking
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP / STEERING_TIME)
driver.SetThrottleDelta(RENDER_STEP / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_STEP / BRAKING_TIME)
driver.Initialize()


# === Review recording setup === capture video and physics data only during validation
frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    # === Main loop === real-time render cadence with full vehicle subsystem updates
    while vis.Run() and system.GetChTime() < SIM_END:
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

except (RuntimeError, ValueError, OSError, IOError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
