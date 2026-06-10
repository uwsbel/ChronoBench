"""
BMW E90 Sedan on Rigid Terrain — PyChrono 9.0.x / Irrlicht

Models a BMW E90 sedan driving on a flat rigid terrain with NSC contact.
System type: ChSystemNSC (owned by the veh.BMW_E90 wrapper).
Main bodies: BMW E90 chassis, suspension/steering/drivetrain subsystems,
             four wheels with TMEASY tires, rigid terrain patch.
Expected behaviour: vehicle accelerates and steers under interactive driver
                    control from the Irrlicht window (chase-camera view).
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr  # noqa: F401 — used via veh visual system
import pychrono.vehicle as veh

# === Constants ===
STEP_SIZE = 1e-3          # physics time step (s)
SIM_END   = 20.0          # simulation end time (s)
TERRAIN_LENGTH = 300.0    # terrain patch X extent (m)
TERRAIN_WIDTH  = 300.0    # terrain patch Y extent (m)

RENDER_FPS    = 50.0
RENDER_STEPS  = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# Vehicle initial position: flat terrain top is z=0; BMW E90 suspension ref ~0.5 m
SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest (m)
INIT_POS = chrono.ChVector3d(0.0, 0.0, SUSPENSION_REF_HEIGHT)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)  # identity (facing +X)

# === Data paths (required truth components for all catalog-vehicle demos) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)   # MANDATORY — fixed chassis never moves
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)   # prompt: TMEASY tire model
vehicle.SetTireStepSize(STEP_SIZE)
vehicle.Initialize()

# === System & bodies (created by the veh.BMW_E90 wrapper) ===
sys     = vehicle.GetSystem()                    # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()               # cache: fetched once, reused below
# wheels/spindles: vehicle.GetVehicle().GetAxle(i)...  (used by footprint assert below)
# joints: suspension + steering created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Visualization types (after Initialize)
vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain (rigid, NSC) ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()   # NSC matches ChContactMethod_NSC
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Wheel-bottom footprint assertion (guard spawn height) ===
TIRE_RADIUS = 0.32   # approximate BMW E90 tire radius (m)
ZTOL = 0.08          # allowed wheel-bottom clearance vs terrain (m)

veh_obj  = vehicle.GetVehicle()   # cache: fetched once for assertions
sp_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        sp_world.append(veh_obj.GetSpindlePos(axle_idx, side))

wheel_bottom_z = min(p.z for p in sp_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain z=0; raise SUSPENSION_REF_HEIGHT by "
    f"{-wheel_bottom_z:.3f} m"
)

# === Irrlicht visualization (vehicle-specific) ===
# veh.ChWheeledVehicleVisualSystemIrrlicht: configure BEFORE Initialize,
# add scene elements AFTER Initialize (Irrlicht call order).
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("BMW E90 Sedan — Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()   # vehicle truths use directional light, NOT AddTypicalLights
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver (interactive, scored-core default) ===
# ChInteractiveDriverIRR takes the visual system (not the vehicle).
render_step_size = 1.0 / RENDER_FPS   # precomputed once — used for delta calc
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)   # 1 s to max steering
driver.SetThrottleDelta(render_step_size / 1.0)   # 1 s to max throttle
driver.SetBrakingDelta(render_step_size / 0.3)    # 0.3 s to max braking
driver.Initialize()
vis.AttachDriver(driver)


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0  # review-only frame counter kept outside try so finally can flush

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (order is fixed: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)   # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
