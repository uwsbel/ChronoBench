"""
HMMWV on Highway Mesh Terrain — Single Collision Mesh Patch + Visual Mesh Overlay

Simulates an HMMWV vehicle driving on a single rigid terrain patch whose collision
geometry is defined by the Highway_col.obj mesh and whose visual appearance is
provided by Highway_vis.obj (added as a ChVisualShapeTriangleMesh to the terrain
ground body). NSC contact method. Interactive keyboard driver. Z-up, 30-second run.

System: ChSystemNSC (owned by HMMWV_Full wrapper)
Contact: NSC, Bullet collision system
Terrain: RigidTerrain with one mesh-based patch (Highway_col.obj collision,
         Highway_vis.obj visual overlay)
Vehicle spawn: ChVector3d(6, -70, 0.5)
"""

# === Imports ===
import math
import os
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants ===
# Vehicle spawn position (prompt-specified)
INIT_X = 6.0
INIT_Y = -70.0
INIT_Z = 0.5

# Simulation timing
TIME_STEP = 2e-3        # 2 ms physics step
SIM_END = 30.0          # 30 seconds
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Terrain contact material parameters
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# Highway mesh paths (located in synchrono/meshes in the conda env data tree)
_DATA_ROOT = chrono.GetChronoDataPath()
HIGHWAY_COL_MESH = _DATA_ROOT + "synchrono/meshes/Highway_col.obj"
HIGHWAY_VIS_MESH = _DATA_ROOT + "synchrono/meshes/Highway_vis.obj"

# === Vehicle setup ===
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity — aligned with X axis

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)       # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)    # no wrapper chassis collision
hmmwv.SetChassisFixed(False)                              # MANDATORY — must allow motion
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)              # TMEASY for good traction
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                   # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()             # cache: main chassis rigid body
# wheels/spindles accessed via hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain below
# joints: suspension + steering links created inside the HMMWV_Full wrapper

# Collision system — REQUIRED for vehicle/terrain contact scenes
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Visualization types (set after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain — single mesh patch (Highway_col.obj) + visual overlay ===
terrain = veh.RigidTerrain(system)

# NSC contact material for the terrain patch
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)

# Single mesh-based collision patch (replaces the four flat patches)
# AddPatch overload: (mat, ChCoordsysd, mesh_file, connected_mesh, sweep_sphere_radius, visualization)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,           # position: identity at origin
    HIGHWAY_COL_MESH,          # collision mesh
    True,                      # connected mesh
    0.01,                      # contact material thickness (sweep sphere radius)
    True,                      # enable default visualization from the col mesh
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

terrain.Initialize()

# === Visual mesh overlay on the terrain ground body ===
# Load the visual highway mesh and attach it to the terrain patch body
highway_vis_trimesh = chrono.ChTriangleMeshConnected()
highway_vis_trimesh.LoadWavefrontMesh(HIGHWAY_VIS_MESH, True, True)

vis_shape = chrono.ChVisualShapeTriangleMesh(highway_vis_trimesh, True)
vis_shape.SetName("highway_visual")
vis_shape.SetMutable(False)  # static terrain mesh — no deformation

# Attach the visual shape to the terrain patch ground body
terrain_body = patch.GetGroundBody()
terrain_body.AddVisualShape(vis_shape, chrono.ChFramed())

# === Verify wheel-bottom clearance after Initialize ===
TIRE_RADIUS = 0.47    # HMMWV TMEASY approximate tire radius (m)
ZTOL = 0.15           # allowed clearance tolerance (generous for mesh terrain)

vehicle_obj = hmmwv.GetVehicle()  # cache: inner vehicle object
spindle_world = []
for axle_idx in range(vehicle_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = vehicle_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"Vehicle wheel-bottom z={wheel_bottom_z:.3f} too far below z=0 by "
    f"{-wheel_bottom_z:.3f} m; raise INIT_Z by that amount."
)

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Highway Mesh Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver — interactive keyboard driver (scored core) ===
render_step_size = 1.0 / RENDER_FPS
steering_time = 1.0   # seconds to reach max steering
throttle_time = 1.0   # seconds to reach max throttle
braking_time = 0.3    # seconds to reach max brake

driver = veh.ChInteractiveDriver(hmmwv.GetVehicle())  # takes ChVehicle in this 9.0.0 build
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        sim_time = hmmwv.GetSystem().GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)


        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)    # advances the wrapper-owned ChSystem
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
