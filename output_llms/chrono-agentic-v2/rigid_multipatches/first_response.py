"""
HMMWV on Rigid Multi-Patch Terrain — PyChrono 9.0.x / Irrlicht

Models a full HMMWV (High Mobility Multipurpose Wheeled Vehicle) driving on a
RigidTerrain composed of multiple heterogeneous patches:
  - Two flat patches with different surface textures (road asphalt, grass tile)
  - One mesh-based patch (bump OBJ mesh)
  - One heightmap-based patch (varying elevation from a BMP file)

System type : NSC (rigid terrain → ChContactMethod_NSC)
Driver      : ChInteractiveDriverIRR (keyboard / real-time interactive)
Expected    : Vehicle spawns on the flat patch, can be driven interactively
              across the different surface types; bump and heightmap patches
              produce visible elevation changes.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Named constants ===
step_size       = 2e-3          # physics time step (s)
sim_end         = 20.0          # simulation end time (s)
render_fps      = 50.0
render_steps    = math.ceil(1.0 / (render_fps * step_size))  # precomputed once

# Terrain patch layout
TERRAIN_LENGTH  = 100.0         # X extent of each flat patch (m)
TERRAIN_WIDTH   = 50.0          # Y extent of each flat patch (m)

# Vehicle spawn
INIT_X          = -25.0
INIT_Y          = 0.0
SUSPENSION_REF_HEIGHT = 0.5     # chassis origin above wheel-bottom at rest (HMMWV)
INIT_Z          = 0.0 + SUSPENSION_REF_HEIGHT
TIRE_RADIUS     = 0.33          # approximate HMMWV tire radius (m)

# === Data paths (mandatory for catalog vehicle truths) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)      # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                            # MANDATORY
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)             # identity (facing +X)
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)             # TMEASY for good grip
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                             # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()                        # cache: fetched once, reused
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain patch bodies below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types (after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# Validate wheel placement after Initialize (wheel bottom should be at or above z=0)
ZTOL = 0.1
veh_obj = hmmwv.GetVehicle()
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        sp_pos = veh_obj.GetSpindlePos(axle_idx, side)
        wheel_bottom = sp_pos.z - TIRE_RADIUS
        assert wheel_bottom >= -ZTOL, (
            f"Wheel bottom z={wheel_bottom:.3f} is below terrain; "
            f"raise SUSPENSION_REF_HEIGHT by {-ZTOL - wheel_bottom:.3f} m"
        )

# === Terrain (RigidTerrain with multiple patches) ===
terrain = veh.RigidTerrain(system)

# Shared NSC contact material for all patches
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Patch 1: flat asphalt road (centered near spawn)
patch1 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-25.0, 0.0, 0.0), chrono.QUNIT),
    TERRAIN_LENGTH, TERRAIN_WIDTH
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 2: flat grass tile (adjacent in +X direction)
patch2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(75.0, 0.0, 0.0), chrono.QUNIT),
    TERRAIN_LENGTH, TERRAIN_WIDTH
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.4, 0.6, 0.3))

# Patch 3: mesh-based bump patch (OBJ mesh for a road bump)
patch3 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25.0, 0.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj")
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 10, 10)
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.5))

# Patch 4: heightmap-based patch for varying elevations
patch4 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25.0, -60.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    60.0, 60.0,   # length, width
    -1.0, 5.0     # height range [hMin, hMax]
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 60, 60)
patch4.SetColor(chrono.ChColor(0.6, 0.5, 0.4))

terrain.Initialize()

# === Visualization (ChWheeledVehicleVisualSystemIrrlicht) ===
# Vehicle demos use AddLightDirectional(), not AddTypicalLights()
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Rigid Multi-Patch Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()                                        # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                               # vehicle-truth standard
vis.AttachVehicle(hmmwv.GetVehicle())                   # attach AFTER Initialize

# === Driver (interactive — scored-core default for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0   # s to go 0 -> +1 steering
throttle_time = 1.0   # s to go 0 -> +1 throttle
braking_time  = 0.3   # s to go 0 -> +1 braking
render_step_size = 1.0 / render_fps  # precomputed once
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()                       # cache: fetched once per outer step

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize in order: driver -> terrain -> vehicle -> vis
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance all subsystems
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)    # advances the wrapper-owned ChSystem
        vis.Advance(step_size)


        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:              # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
