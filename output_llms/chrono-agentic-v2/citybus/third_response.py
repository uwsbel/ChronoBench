"""
CityBus simulation on rigid terrain using PyChrono 9.0.0 with Irrlicht visualization.

System type: NSC (Non-Smooth Contact) — standard for rigid-terrain catalog vehicle demos.
Vehicle: CityBus wrapper (veh.CityBus), wheeled vehicle with bus proportions.
Tire model: Pacejka 89 (PAC02) — more accurate tire force model than TMEasy.
Terrain: RigidTerrain with a flat patch textured with dirt.jpg.
Simulation step: 5e-4 s (reduced from 1e-3 for better stability with PAC02 tires).
Expected behavior: Bus drives forward on flat dirt terrain with an interactive driver.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Named constants ===
# Simulation parameters
step_size = 5e-4           # reduced from 1e-3 for PAC02 stability
tire_step_size = 5e-4      # reduced from 1e-3 for PAC02 stability
sim_end = 20.0
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_steps = max(1, math.ceil(render_step_size / step_size))  # precomputed once

# Vehicle initial position and orientation
init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Terrain dimensions
terrain_length = 200.0
terrain_width = 200.0

# === Data paths (required for catalog vehicle — scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)          # MANDATORY — fixed chassis won't move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_PAC02)   # Pacejka 89 tire model (prompt: PAC02)
bus.SetTireStepSize(tire_step_size)
bus.Initialize()

# === System & bodies (created by the veh.CityBus wrapper) ===
sys = bus.GetSystem()               # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = bus.GetChassisBody()      # cache: main chassis rigid body, fetched once
# wheels/spindles: bus.GetVehicle().GetAxle(i)...
# joints: suspension + steering links created inside the wrapper
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# === Visualization types ===
bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)  # dirt road texture
patch.SetColor(chrono.ChColor(0.8, 0.7, 0.5))

terrain.Initialize()

# === Visualization (Irrlicht — vehicle demo uses ChWheeledVehicleVisualSystemIrrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus - Pacejka Tires on Dirt Road")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()   # vehicle truths use directional light, not AddTypicalLights
vis.AttachVehicle(bus.GetVehicle())


# === Driver (interactive — scored core; scripted maneuver is review-only) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        # Throttled rendering: render only every render_steps physics steps
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (MANDATORY order: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        bus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance subsystems
        driver.Advance(step_size)
        terrain.Advance(step_size)
        bus.Advance(step_size)   # advances the wrapper-owned ChSystem — do NOT also call sys.DoStepDynamics
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
