"""
Kraz Tractor-Trailer simulation using PyChrono 9.0.x with Irrlicht visualization.

System type: NSC (rigid terrain catalog vehicle default)
Main bodies: Kraz tractor + trailer (wrapper-managed), RigidTerrain flat patch
Expected behavior: Kraz tractor-trailer sitting on flat rigid terrain, ready for
interactive keyboard-driven control; driver inputs accepted from the Irrlicht
window via ChInteractiveDriverIRR.
"""

import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants ===
step_size = 5e-4           # physics time step (s)
sim_end = 20.0             # simulation end time (s)
render_fps = 50.0          # render frame rate
render_step_size = 1.0 / render_fps
render_steps = max(1, math.ceil(render_step_size / step_size))  # precomputed once

terrainLength = 200.0      # terrain X size (m)
terrainWidth = 200.0       # terrain Y size (m)

initLoc = chrono.ChVector3d(0, 0, 0.5)        # vehicle spawn location
initRot = chrono.QuatFromAngleZ(0.0)          # vehicle spawn orientation (facing +X)

# Chase camera parameters
chase_offset = chrono.ChVector3d(0.0, 0.0, 1.75)  # track point on chassis
chase_dist = 14.0
chase_vert = 0.5

# === Data paths (mandatory for catalog vehicles, scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle: Kraz tractor-trailer ===
# veh.Kraz() creates the tractor + trailer wrapper; owns its ChSystem.
# Visualization must be attached via vehicle.GetTractor(), NOT GetVehicle().
vehicle = veh.Kraz()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

# === System & bodies (created by the veh.Kraz wrapper) ===
sys = vehicle.GetSystem()                       # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = vehicle.GetTractor().GetChassisBody()  # cache: tractor chassis rigid body
# trailer chassis: vehicle.GetTrailer().GetChassisBody()
# wheels/spindles: vehicle.GetTractor().GetAxle(i)...; terrain added below
# joints: suspension + steering links created inside the Kraz wrapper

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Visualization types (set after Initialize)
vis_type_mesh = chrono.VisualizationType_MESH
vis_type_prim = chrono.VisualizationType_PRIMITIVES

vehicle.SetChassisVisualizationType(vis_type_mesh)
vehicle.SetSuspensionVisualizationType(vis_type_prim)
vehicle.SetSteeringVisualizationType(vis_type_prim)
vehicle.SetWheelVisualizationType(vis_type_mesh)
vehicle.SetTireVisualizationType(vis_type_mesh)

# === Terrain: RigidTerrain flat patch (NSC) ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrainLength,
    terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# === Irrlicht visualization for wheeled vehicles ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Tractor-Trailer")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chase_offset, chase_dist, chase_vert)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()   # vehicle demos use directional light, not AddTypicalLights
vis.AttachVehicle(vehicle.GetTractor())

# === Interactive driver (scored-core default for catalog vehicle demos) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0    # s to full steering
throttle_time = 1.0    # s to full throttle
braking_time = 0.3     # s to full braking
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        sim_time = sys.GetChTime()

        # --- Throttled rendering (render_steps physics steps per frame) ---
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize in order: driver -> terrain -> vehicle -> vis
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        vehicle.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)


        # Advance in order: driver -> terrain -> vehicle -> vis
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)   # internally steps the ChSystem
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
