"""Gator wheeled vehicle on a multi-patch rigid terrain (NSC, Z-up).

This script models a John Deere Gator (veh.Gator catalog wrapper, TMEASY tires,
SHAFTS brake) driving on a veh.RigidTerrain that is split into FOUR distinct
patches, each with its own contact material and texture:
  * patch 1 — flat tiled patch (tile4 texture),
  * patch 2 — flat concrete patch, slightly raised,
  * patch 3 — a bump patch built from the bump.obj mesh (uneven dirt surface),
  * patch 4 — a height-mapped grass patch (bump64.bmp) that forms an inclined
    rise so the Gator's gradability can be tested.
The vehicle is driven by an interactive Irrlicht driver. Expected behavior: the
Gator rests level on the tiled patch and, under throttle, climbs the height-map
slope demonstrating gradability over the bump and graded patches.
"""

import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === step sizes, timing, spawn pose, terrain patch layout
step_size = 1e-3                       # integration step (s)
tire_step_size = step_size             # TMEASY tire sub-step (s)
sim_end = 15.0                         # bounded recording horizon (s)
render_fps = 50.0
render_step_size = 1.0 / render_fps    # precomputed once: render cadence (s)

init_loc = chrono.ChVector3d(-10, -2, 0.6)      # spawn on the tiled patch
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Four-patch terrain layout (centres in world XY, sizes in m).
PATCH1_CTR = chrono.ChVector3d(-16, 0, 0.0)     # tiled flat patch
PATCH2_CTR = chrono.ChVector3d(16, 0, 0.15)     # concrete flat patch (raised)
PATCH3_CTR = chrono.ChVector3d(0, -42, 0.0)     # bump.obj mesh patch
PATCH4_CTR = chrono.ChVector3d(0, 42, 0.0)      # height-map grade patch


# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === Gator catalog wrapper owns its ChSystemNSC; build + initialize
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)          # rigid terrain -> NSC
gator.SetChassisFixed(False)                                # MANDATORY: chassis must move
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)                 # deformable-contact tire
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_NONE)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.Gator wrapper) ===
system = gator.GetSystem()                       # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody()                 # cache: main chassis rigid body, reused below
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for contact
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# === Terrain === one RigidTerrain split into four textured patches
terrain = veh.RigidTerrain(system)

# Patch 1 — flat tiled patch (the spawn patch).
patch1_mat = chrono.ChContactMaterialNSC()
patch1_mat.SetFriction(0.9)
patch1_mat.SetRestitution(0.01)
patch1 = terrain.AddPatch(patch1_mat, chrono.ChCoordsysd(PATCH1_CTR, chrono.QUNIT), 32, 20)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

# Patch 2 — flat concrete patch, slightly raised.
patch2_mat = chrono.ChContactMaterialNSC()
patch2_mat.SetFriction(0.9)
patch2_mat.SetRestitution(0.01)
patch2 = terrain.AddPatch(patch2_mat, chrono.ChCoordsysd(PATCH2_CTR, chrono.QUNIT), 32, 30)
patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)

# Patch 3 — bump patch from the bump.obj mesh (uneven dirt surface).
patch3_mat = chrono.ChContactMaterialNSC()
patch3_mat.SetFriction(0.9)
patch3_mat.SetRestitution(0.01)
patch3 = terrain.AddPatch(patch3_mat, chrono.ChCoordsysd(PATCH3_CTR, chrono.QUNIT),
                          veh.GetDataFile("terrain/meshes/bump.obj"))
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# Patch 4 — height-mapped grass patch (graded rise for gradability testing).
patch4_mat = chrono.ChContactMaterialNSC()
patch4_mat.SetFriction(0.9)
patch4_mat.SetRestitution(0.01)
patch4 = terrain.AddPatch(patch4_mat, chrono.ChCoordsysd(PATCH4_CTR, chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/bump64.bmp"), 64.0, 64.0, 0.0, 3.0)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6.0, 6.0)

terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase cam + sky + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Multi-Patch Terrain")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()                          # vehicle truths use a directional light
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())

# === Driver === interactive Irrlicht driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0     # s to ramp steering 0 -> +1
throttle_time = 1.0     # s to ramp throttle 0 -> +1
braking_time = 0.3      # s to ramp braking 0 -> +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Main loop === real-time synchronize/advance of the full subsystem stack
render_steps = math.ceil(render_step_size / step_size)   # precomputed once: steps per frame
gator.GetVehicle().EnableRealtime(True)


step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:              # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)                   # match wall-clock to sim time
except (RuntimeError, ValueError) as exc:                # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
