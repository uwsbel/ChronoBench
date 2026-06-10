"""
ARTcar vehicle simulation on rigid terrain with Irrlicht visualization.

System type: ChSystemNSC (NSC contact, owned by the veh.ARTcar wrapper).
Main bodies: ARTcar chassis + 4 wheel spindles (wrapper-managed), rigid terrain patch.
Expected behavior: ARTcar vehicle initialized at origin, controlled interactively
via keyboard (steering, throttle, braking). Rigid flat terrain with a custom
texture. Simulation runs at 50 fps real time with a chase camera.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
# Simulation timing
STEP_SIZE = 1e-3           # physics time step (s)
SIM_END   = 30.0           # simulation end time (s)
RENDER_FPS = 50.0          # target render rate (Hz)

# Terrain geometry
TERRAIN_LENGTH = 300.0     # terrain X extent (m)
TERRAIN_WIDTH  = 300.0     # terrain Y extent (m)

# Vehicle init
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)   # spawn above terrain
INIT_ROT = chrono.QuatFromAngleZ(0.0)          # facing +X

# Derived render cadence — precomputed once
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# === Data paths (truth-required trio) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle ===
artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)
artcar.SetChassisCollisionType(veh.CollisionType_NONE)
artcar.SetChassisFixed(False)
artcar.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
artcar.SetTireType(veh.TireModelType_RIGID)
artcar.SetTireStepSize(STEP_SIZE)
artcar.Initialize()

# Visualization types (called after Initialize); enum lives in veh in this 9.0.0 build
artcar.SetChassisVisualizationType(veh.VisualizationType_MESH)
artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetWheelVisualizationType(veh.VisualizationType_MESH)
artcar.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.ARTcar wrapper) ===
system = artcar.GetSystem()            # ChSystemNSC owned by the wrapper  # cache: fetched once, reused every step
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = artcar.GetChassisBody()      # cache: fetched once, reused every step
# wheels/spindles: artcar.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

print("VEHICLE MASS: ", artcar.GetVehicle().GetMass())

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

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 3.5, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(artcar.GetVehicle())

# === Driver (interactive — scored-core default for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0   # s from 0 → max steering
throttle_time = 1.0   # s from 0 → max throttle
braking_time  = 0.3   # s from 0 → max braking
render_step = 1.0 / RENDER_FPS

driver.SetSteeringDelta(render_step / steering_time)
driver.SetThrottleDelta(render_step / throttle_time)
driver.SetBrakingDelta(render_step / braking_time)
driver.Initialize()


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

        # <<< review-only

        driver.Synchronize(time)
        terrain.Synchronize(time)
        artcar.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        artcar.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
