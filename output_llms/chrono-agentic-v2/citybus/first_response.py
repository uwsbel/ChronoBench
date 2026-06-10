"""
CityBus simulation on RigidTerrain using PyChrono with Irrlicht visualization.

System: NSC (Non-Smooth Contact) — standard for rigid-terrain catalog vehicles.
Vehicle: veh.CityBus() wrapper on a flat RigidTerrain patch with a tile texture.
Driver: ChInteractiveDriverIRR — real-time interactive keyboard control.
Expected behavior: Bus initializes on flat terrain; interactive driver provides
steering/throttle/braking via keyboard; chase camera follows the vehicle.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Constants ===
STEP_SIZE = 5e-4          # physics time step (s)
SIM_END = 100.0           # simulation end time (s)
RENDER_FPS = 50.0         # render / frame-capture rate (Hz)
TERRAIN_LENGTH = 600.0    # terrain patch length (m)
TERRAIN_WIDTH = 600.0     # terrain patch width (m)
INIT_LOC = chrono.ChVector3d(0, 0, 0.5)
INIT_ROT = chrono.QuatFromAngleZ(0.0)   # heading along +X

# === Data paths (truth-required pair for catalog vehicle demos) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(STEP_SIZE)
bus.Initialize()

# === System & bodies (created by the veh.CityBus wrapper) ===
system = bus.GetSystem()               # ChSystemNSC owned by the wrapper
chassis = bus.GetChassisBody()         # cache: fetched once, reused every step
# wheels/spindles: bus.GetVehicle().GetAxle(i).m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# Visualization types — chrono.* namespace (not veh.*)
bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
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

# === Visualization — full Irrlicht scene (Initialize FIRST, then scene elements) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(bus.GetVehicle())

# === Driver — interactive (truth-faithful ChInteractiveDriverIRR form) ===
driver = veh.ChInteractiveDriverIRR(vis)
STEERING_TIME = 1.0          # s to reach max steering
THROTTLE_TIME = 1.0          # s to reach max throttle
BRAKING_TIME = 0.3           # s to reach max braking
render_step_size = 1.0 / RENDER_FPS   # precomputed once
driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# Precomputed render cadence
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_every == 0:
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

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
