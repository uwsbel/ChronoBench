"""
FEDA Vehicle on Rigid Terrain — PyChrono simulation.

System type: NSC (Non-Smooth Contact)
Vehicle: FED-Alpha (veh.FEDA) with PAC02 tires, NSC contact method.
Terrain: RigidTerrain with a single flat patch and custom texture.
Driver: Interactive (ChInteractiveDriverIRR) for real-time keyboard control.
Visualization: ChWheeledVehicleVisualSystemIrrlicht with chase camera.
Expected behavior: FEDA vehicle rests on flat terrain; the user steers/accelerates
via keyboard inputs. The visualization follows the vehicle from the rear.
"""

import os, math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants ===
STEP_SIZE        = 2e-3        # physics time step (s)
SIM_END          = 20.0        # simulation end time (s)
RENDER_FPS       = 50.0        # target render frames per second
TERRAIN_LENGTH   = 200.0       # terrain patch length (m)
TERRAIN_WIDTH    = 100.0       # terrain patch width (m)
INIT_LOC         = chrono.ChVector3d(0, 0, 0.5)   # initial chassis origin (m)
INIT_ROT         = chrono.ChQuaterniond(1, 0, 0, 0)

render_steps     = max(1, math.ceil(1.0 / (RENDER_FPS * STEP_SIZE)))  # steps per frame; precomputed once

# === Data paths (required for all catalog-vehicle truths) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)       # rigid terrain → NSC
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)                             # MANDATORY — fixed chassis won't move
feda.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)               # PAC02 tire for rigid terrain
feda.SetTireStepSize(STEP_SIZE)
feda.Initialize()

# === System & bodies (created by the veh.FEDA wrapper) ===
system  = feda.GetSystem()                       # ChSystemNSC owned by the wrapper
chassis = feda.GetChassisBody()                  # cache: main chassis rigid body; fetched once
# Wheels/spindles: feda.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# Joints: suspension + steering links created inside the FEDA wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED after Initialize
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

# === Visualization types (called after Initialize; VisualizationType is in veh namespace) ===
feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain  = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()        # NSC matches vehicle contact method
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

# === Irrlicht visualization (vehicle visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA Vehicle on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # follow from behind
vis.Initialize()                                                # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                       # vehicle truth uses directional light
vis.AttachVehicle(feda.GetVehicle())

# === Driver (interactive, keyboard-controlled) ===
steering_time = 1.0    # s to reach max steering
throttle_time = 1.0    # s to reach max throttle
braking_time  = 0.3    # s to reach max braking

driver = veh.ChInteractiveDriverIRR(vis)   # takes the visual system, not the vehicle
driver.SetSteeringDelta(1.0 / (RENDER_FPS * steering_time))
driver.SetThrottleDelta(1.0 / (RENDER_FPS * throttle_time))
driver.SetBrakingDelta(1.0 / (RENDER_FPS * braking_time))
driver.Initialize()

# === Recording setup (review-only) ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        # Throttled rendering — render once per frame, not per step
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Driver inputs (must read BEFORE Synchronize)
        driver_inputs = driver.GetInputs()


        # Synchronize full subsystem stack in fixed order
        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance all subsystems (vehicle.Advance steps the wrapped system — do NOT also call DoStepDynamics)
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        feda.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)   # keep wall-clock ≈ sim time

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
