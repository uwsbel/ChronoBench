"""
MAN 10t Truck on Rigid Terrain — PyChrono 9.0.0 (NSC, Irrlicht)
================================================================
Models a MAN 10t truck driving on a flat rigid terrain with a TMEASY tire model.
The vehicle wrapper owns the ChSystemNSC. An interactive IRR driver lets the user
steer, accelerate, and brake in real time. Chase camera follows the chassis.
Expected behavior: truck rests on terrain, responds to keyboard controls,
accelerates forward under throttle, steers left/right, brakes to a stop.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Simulation parameters ===
step_size = 1e-3        # physics time step (s)
sim_end   = 20.0        # simulation end time (s)
render_fps = 50.0       # Irrlicht render rate (Hz)
render_steps = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once

terrainLength = 300.0   # terrain X extent (m)
terrainWidth  = 300.0   # terrain Y extent (m)

# Vehicle initial position (flat terrain, z=0 is ground)
SUSPENSION_REF_HEIGHT = 0.53   # chassis origin above wheel-bottom at rest (MAN 10t)
init_loc = chrono.ChVector3d(0.0, 0.0, SUSPENSION_REF_HEIGHT)
init_rot = chrono.QuatFromAngleZ(0.0)  # facing +X

# === Data paths (truth-required for all catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
truck = veh.MAN_10t()
truck.SetContactMethod(chrono.ChContactMethod_NSC)
truck.SetChassisCollisionType(veh.CollisionType_NONE)
truck.SetChassisFixed(False)                              # MANDATORY — fixed chassis won't move
truck.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
truck.SetTireType(veh.TireModelType_TMEASY)               # prompt: TMEASY tire model
truck.SetTireStepSize(step_size)
truck.Initialize()

# === System & bodies (created by the veh.MAN_10t wrapper) ===
sys     = truck.GetSystem()               # ChSystemNSC owned by the wrapper
chassis = truck.GetChassisBody()          # main chassis rigid body; cache: fetched once
# axles/spindles: truck.GetVehicle().GetAxles(); terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Visualization types (after Initialize)
truck.SetChassisVisualizationType(chrono.VisualizationType_MESH)
truck.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(chrono.VisualizationType_MESH)
truck.SetTireVisualizationType(chrono.VisualizationType_MESH)

print("VEHICLE MASS: ", truck.GetVehicle().GetMass())   # truth-required diagnostic

# === Terrain ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()   # NSC matches ChContactMethod_NSC
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

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 10t Truck — Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                           # vehicle demos use directional light
vis.AttachVehicle(truck.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0    # s to reach max steering
throttle_time = 1.0    # s to reach max throttle
braking_time  = 0.3    # s to reach max braking

driver.SetSteeringDelta(1.0 / (render_fps * steering_time))   # precomputed once
driver.SetThrottleDelta(1.0 / (render_fps * throttle_time))
driver.SetBrakingDelta(1.0 / (render_fps * braking_time))
driver.Initialize()

vis.AttachDriver(driver)

# Wheel-bottom Z assert: verify vehicle rests on terrain (not sunken)
TIRE_RADIUS = 0.55    # approximate MAN 10t tire radius (m)
ZTOL = 0.10
veh_obj = truck.GetVehicle()        # cache: fetched once
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise SUSPENSION_REF_HEIGHT by {abs(wheel_bottom_z):.3f} m"
)

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        truck.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        truck.Advance(step_size)        # advances the wrapper-owned ChSystem
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
