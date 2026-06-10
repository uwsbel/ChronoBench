"""
FEDA vehicle simulation on rigid terrain with Irrlicht visualization.

System type: NSC (Non-Smooth Contact)
Vehicle: FEDA (FED-Alpha) wheeled vehicle wrapper
Terrain: RigidTerrain with custom texture and flat patch
Driver: ChInteractiveDriverIRR (interactive keyboard control)
Expected behavior: Vehicle rests on flat terrain; user steers/throttles
interactively at 50 FPS render rate. A review-only block provides a scripted
open-loop maneuver for the RUN-stage validation video.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Named constants ===
STEP_SIZE = 2e-3           # physics time step (s)
SIM_END = 20.0             # simulation end time (s)
RENDER_FPS = 50.0          # render cadence (frames per second)
TERRAIN_LENGTH = 200.0     # terrain patch length (m)
TERRAIN_WIDTH = 200.0      # terrain patch width (m)
INIT_X = 0.0               # vehicle spawn X (m)
INIT_Y = 0.0               # vehicle spawn Y (m)
INIT_Z = 0.5               # chassis origin height above terrain at rest (m)
STEERING_TIME = 1.0        # seconds to reach max steering
THROTTLE_TIME = 1.0        # seconds to reach max throttle
BRAKING_TIME = 0.3         # seconds to reach max braking

render_steps = max(1, math.ceil(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# === Data paths (truth-required for catalog-vehicle Reference score) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)        # NSC for rigid terrain
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)                              # MANDATORY — fixed won't move
feda.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
    chrono.ChQuaterniond(1, 0, 0, 0)
))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(STEP_SIZE)
feda.Initialize()

# === System & bodies (created by the veh.FEDA wrapper) ===
system = feda.GetSystem()                                # ChSystemNSC owned by wrapper
chassis = feda.GetChassisBody()                          # cache: fetched once, reused
# wheels/spindles: feda.GetVehicle().GetAxle(i)... ; terrain patch body below
# joints: suspension + steering links created inside the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Visualization types (set after Initialize)
feda.SetChassisVisualizationType(chrono.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(chrono.VisualizationType_MESH)
feda.SetTireVisualizationType(chrono.VisualizationType_MESH)

print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()               # NSC to match contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (Irrlicht vehicle visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                                        # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                               # vehicle truths use directional light
vis.AttachVehicle(feda.GetVehicle())

# === Driver (interactive — scored-core truth shape) ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(1.0 / (RENDER_FPS * STEERING_TIME))   # precomputed once
driver.SetThrottleDelta(1.0 / (RENDER_FPS * THROTTLE_TIME))
driver.SetBrakingDelta(1.0 / (RENDER_FPS * BRAKING_TIME))
driver.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        feda.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
