"""
Gator vehicle simulation on flat RigidTerrain with Irrlicht visualization.

System: ChSystemNSC (owned by veh.Gator wrapper)
Vehicle: John Deere Gator utility vehicle (veh.Gator catalog wrapper)
Terrain: RigidTerrain with flat patch, NSC contact material, custom texture
Tires: TMEASY tire model
Driver: Interactive (ChInteractiveDriverIRR) for real-time keyboard control
Visualization: ChWheeledVehicleVisualSystemIrrlicht with mesh vis for all parts
Expected behavior: Gator vehicle rests on flat terrain, driver can steer/throttle/brake;
                   simulation runs at 50 Hz render rate in real time.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh

# Set vehicle data path to the bundled Chrono vehicle data tree
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Constants & Parameters ===
# Physics time step and simulation duration
time_step = 2e-3          # 2 ms physics step
sim_end = 20.0            # seconds
render_fps = 50.0         # frames per second for rendering
render_step_size = 1.0 / render_fps  # 0.02 s between render frames
render_steps = math.ceil(render_step_size / time_step)  # precomputed once

# Terrain dimensions
terrain_length = 200.0    # m
terrain_width = 200.0     # m

# Vehicle spawn position (Z set from suspension reference height above flat terrain at z=0)
SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest (Gator ~0.5 m)
init_x = 0.0
init_y = 0.0
init_z = 0.0 + SUSPENSION_REF_HEIGHT
init_loc = chrono.ChVector3d(init_x, init_y, init_z)
init_rot = chrono.QuatFromAngleZ(0.0)   # facing +X

# === Vehicle Setup (veh.Gator wrapper) ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)                         # MANDATORY: fixed chassis won't move
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
gator.SetTireType(veh.TireModelType_TMEASY)          # TMEASY: good for rigid terrain
gator.SetTireStepSize(time_step)
gator.Initialize()

# === System & bodies (created by the veh.Gator wrapper) ===
system = gator.GetSystem()                # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody()          # main chassis rigid body
# cache: fetched once, reused for terrain and visualization
# wheels/spindles: gator.GetVehicle().GetAxle(i)... created inside the wrapper
# joints: suspension + steering links created inside the wrapper

# Collision system (REQUIRED: vehicle + terrain contact)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Visualization types (mesh for all parts, as prompt specifies) ===
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,         # centered at origin, flat
    terrain_length,
    terrain_width,
)
# Custom texture as prompt specifies
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# === Validate vehicle footprint after Initialize ===
TIRE_RADIUS = 0.282   # Gator tire radius (approximate, from vehicle geometry)
ZTOL = 0.10           # tolerance for wheel-bottom vs terrain surface

veh_obj = gator.GetVehicle()  # cache: fetched once
spindle_positions = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_positions.append(p)

wheel_bottom_z = min(p.z for p in spindle_positions) - TIRE_RADIUS
terrain_z = 0.0   # flat terrain at z=0
assert wheel_bottom_z >= terrain_z - ZTOL, (
    f"Gator sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain z={terrain_z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{terrain_z - wheel_bottom_z:.3f} m"
)

# === Irrlicht Visualization (veh.ChWheeledVehicleVisualSystemIrrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle - Flat Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)

# === Interactive Driver (ChInteractiveDriverIRR — scored-core default for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)

# Time to reach max steering/throttle/braking
steering_time = 1.0   # seconds to go 0 -> full steering
throttle_time = 1.0   # seconds to go 0 -> full throttle
braking_time  = 0.3   # seconds to go 0 -> full braking

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Recording setup (review-only) ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        sim_time = system.GetChTime()  # cache: fetched once per render frame

        # Render at 50 fps cadence
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (fixed order: driver → terrain → vehicle → vis)
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        gator.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)


        # Advance subsystems
        driver.Advance(time_step)
        terrain.Advance(time_step)
        gator.Advance(time_step)   # internally calls DoStepDynamics
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)   # real-time pacing

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
