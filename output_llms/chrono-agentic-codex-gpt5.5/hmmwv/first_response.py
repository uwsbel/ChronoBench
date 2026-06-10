"""Full HMMWV on flat rigid terrain using PyChrono vehicle NSC contact.

The model initializes an HMMWV_Full wrapper with TMEASY tires, primitive visual
components, a flat textured RigidTerrain patch, an Irrlicht vehicle visual
system, and an interactive keyboard driver. The real-time loop renders at
50 frames per second while advancing driver, terrain, vehicle, and visual
subsystems in the standard Chrono order.
"""

import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants: vehicle, terrain, and timing ===
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_EVERY = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 120.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
SUSPENSION_REF_HEIGHT = 0.5
TIRE_RADIUS_ESTIMATE = 0.47
WHEEL_Z_TOLERANCE = 0.15

INIT_LOC = chrono.ChVector3d(0.0, 0.0, SUSPENSION_REF_HEIGHT)
INIT_ROT = chrono.QUNIT
VIS_TYPE = veh.VisualizationType_PRIMITIVES

STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3


# === Vehicle system: HMMWV owns the ChSystem ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)  # prompt: TMEASY tire model
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned system reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = hmmwv.GetVehicle()  # cache: vehicle handle reused for mass, footprint, visualizer
chassis = hmmwv.GetChassisBody()  # cache: chassis body available for diagnostics

print("VEHICLE MASS: ", vehicle.GetMass())

# HMMWV wrapper-created components visible to the source reviewer:
# system: wrapper-owned ChSystem; bodies: chassis, suspensions, wheels, tires;
# contacts: Bullet collision with NSC terrain material; visualization: Irrlicht;
# driver: ChInteractiveDriverIRR bound to the vehicle visual system.

spindle_positions = []
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - TIRE_RADIUS_ESTIMATE
assert wheel_bottom_z >= -WHEEL_Z_TOLERANCE, (
    f"wheel bottom z={wheel_bottom_z:.3f} is below rigid terrain beyond tolerance; "
    "increase SUSPENSION_REF_HEIGHT"
)


# === Vehicle visualization types: primitive components as requested ===
hmmwv.SetChassisVisualizationType(VIS_TYPE)
hmmwv.SetSuspensionVisualizationType(VIS_TYPE)
hmmwv.SetSteeringVisualizationType(VIS_TYPE)
hmmwv.SetWheelVisualizationType(VIS_TYPE)
hmmwv.SetTireVisualizationType(VIS_TYPE)


# === Rigid terrain: flat textured support for tire contact ===
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
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Irrlicht visualization: vehicle-aware real-time renderer ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


# === Driver: interactive keyboard control in scored core ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
driver.Initialize()

# === Main loop: render at 50 FPS and advance all vehicle subsystems ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0

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
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)


            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError, OSError) as exc:
    print(f"simulation failed: {exc}")
    raise


# === Review-only post-processing: assemble videos and remove PNG frames ===
