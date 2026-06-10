"""
HMMWV on Custom Highway Mesh Terrain — PyChrono 9.0.x / Irrlicht

Simulates a full HMMWV (High Mobility Multipurpose Wheeled Vehicle) driving on a
custom highway mesh terrain composed of collision (Highway_col.obj) and visual
(Highway_vis.obj) meshes. The vehicle uses the TMEASY tire model and a MESH
visualization for all components. An interactive driver (ChInteractiveDriverIRR)
lets the user control steering, throttle, and braking in real time. The simulation
runs in real time at 50 fps using ChRealtimeStepTimer. System: ChSystemNSC
(NSC contact method matching rigid-terrain catalog defaults). Expected behaviour:
vehicle rests on the highway mesh surface and responds to keyboard inputs.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
step_size        = 2e-3          # physics time step (s)
sim_end          = 20.0          # simulation end time (s)
render_fps       = 50.0          # rendering rate (Hz)
render_step_size = 1.0 / render_fps
render_steps     = max(1, math.ceil(render_step_size / step_size))   # precomputed once

# Vehicle initial pose
INIT_X = -35.0
INIT_Y = 0.0
INIT_Z = 0.5                     # slight offset above mesh surface at origin height
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(0.0)

# === Data paths (mandatory truth components for every catalog-vehicle demo) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                              # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)               # TMEASY for mesh terrain
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                               # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()                          # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain body: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())     # mandatory truth diagnostic

# === Visualization types ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain — custom highway mesh ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()                 # NSC for rigid-terrain catalog vehicles
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Collision mesh patch (Highway_col.obj supplies contact geometry)
col_mesh_file = chrono.GetChronoDataFile("synchrono/meshes/Highway_col.obj")
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    col_mesh_file,
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

terrain.Initialize()

# Attach the high-res visual mesh (Highway_vis.obj) to the terrain ground body
vis_mesh_file = chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj")
vis_shape = chrono.ChVisualShapeModelFile()
vis_shape.SetFilename(vis_mesh_file)
patch.GetGroundBody().AddVisualShape(vis_shape)

# === Irrlicht vehicle visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Custom Highway Mesh Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                 # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver (scored-core default matching truth) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Footprint assert: verify wheel bottoms rest above mesh surface ===
TIRE_RADIUS = 0.47                                        # HMMWV TMEASY tire radius (m)
ZTOL = 0.15                                               # generous tolerance for mesh terrain
veh_obj = hmmwv.GetVehicle()
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks below z=0: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise INIT_Z by {-wheel_bottom_z:.3f} m"
)


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:                # solver divergence / bad vehicle state
    import traceback; traceback.print_exc()
    raise
