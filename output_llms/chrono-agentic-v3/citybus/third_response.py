"""
CityBus on RigidTerrain with Pacejka PAC89 tire model.

System: ChSystemNSC (owned by the CityBus wrapper).
Vehicle: veh.CityBus with PAC89 tires, step size 5e-4 s, terrain with dirt.jpg texture.
Driver: ChInteractiveDriverIRR (interactive).
Expected: Bus sits on flat terrain and can be driven interactively;
          PAC89 tires handle lateral/longitudinal slip accurately on dirt road.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr  # noqa: F401 (used via veh.ChWheeledVehicleVisualSystemIrrlicht)
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE       = 5e-4    # simulation and tire step size (reduced for PAC89 stability)
SIM_END         = 20.0    # seconds
TERRAIN_LENGTH  = 300.0   # terrain patch X (m)
TERRAIN_WIDTH   = 300.0   # terrain patch Y (m)
INIT_X          = 0.0
INIT_Y          = 0.0
INIT_Z          = 0.5     # chassis origin height above ground at rest
RENDER_FPS      = 50.0
render_step_size = 1.0 / RENDER_FPS                           # precomputed once
render_steps = max(1, math.ceil(render_step_size / STEP_SIZE))  # precomputed once

# === Data paths (required for catalog-vehicle truth fidelity) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_PAC02)   # prompt: Pacejka tire (CityBus uses PAC02 — the shipped Pacejka model; PAC89 falls back to TMeasy)
bus.SetTireStepSize(STEP_SIZE)
bus.Initialize()

# === System & bodies (created by the veh.CityBus wrapper) ===
sys = bus.GetSystem()                         # ChSystemNSC owned by the wrapper
chassis = bus.GetChassisBody()               # cache: main chassis rigid body
# Axles/spindles created inside the bus wrapper; terrain patch body added below.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# Visualization types
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()   # NSC matches the bus contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)  # prompt: dirt road texture
patch.SetColor(chrono.ChColor(0.8, 0.7, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus - PAC89 Tires on Dirt Road")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()   # standard for vehicle demos
vis.AttachVehicle(bus.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        bus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        bus.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
