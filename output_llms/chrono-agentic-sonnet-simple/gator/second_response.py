import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled data
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data

# Vehicle initial position above flat terrain origin
initLoc = chrono.ChVector3d(0, 0, 0.5)                              # spawn slightly above z=0
initRot = chrono.QUNIT                                               # facing +X

gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)                                         # MANDATORY
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY tires
gator.SetTireStepSize(1e-3)
gator.Initialize()

system = gator.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())               # report vehicle mass

# Terrain with 4 patches: flat grass, flat concrete, bump, and slope for gradability
terrain = veh.RigidTerrain(system)

# Shared NSC contact material for all patches
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.01)

# Patch dimensions and layout: 4 patches arranged in a 2x2 grid, each 40x40 m
patch_size = 40.0  # each patch is 40x40 m

# Patch 1: grass texture (front-left, where vehicle starts)
patch1_pos = chrono.ChCoordsysd(chrono.ChVector3d(-patch_size / 2, -patch_size / 2, 0))
patch1 = terrain.AddPatch(patch_mat, patch1_pos, patch_size, patch_size)
patch1.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 20, 20)
patch1.SetColor(chrono.ChColor(0.4, 0.7, 0.3))                     # greenish

# Patch 2: concrete texture (front-right)
patch2_pos = chrono.ChCoordsysd(chrono.ChVector3d(patch_size / 2, -patch_size / 2, 0))
patch2 = terrain.AddPatch(patch_mat, patch2_pos, patch_size, patch_size)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)
patch2.SetColor(chrono.ChColor(0.6, 0.6, 0.6))                     # grey

# Patch 3: dirt texture (back-left) with bump heightmap — for testing bump traversal
patch3_pos = chrono.ChCoordsysd(chrono.ChVector3d(-patch_size / 2, patch_size / 2, 0))
patch3 = terrain.AddPatch(
    patch_mat,
    patch3_pos,
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    patch_size, patch_size, -0.5, 0.5,                              # hMin, hMax (bump shape)
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 20)
patch3.SetColor(chrono.ChColor(0.7, 0.5, 0.3))                     # brownish dirt

# Patch 4: tile texture (back-right) — slope heightmap for gradability testing
patch4_pos = chrono.ChCoordsysd(chrono.ChVector3d(patch_size / 2, patch_size / 2, 0))
patch4 = terrain.AddPatch(
    patch_mat,
    patch4_pos,
    veh.GetDataFile("terrain/height_maps/slope.bmp"),
    patch_size, patch_size, 0.0, 4.0,                              # 0 to 4 m slope
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.5))                     # tile grey

terrain.Initialize()

# Irrlicht visualization — vehicle uses ChWheeledVehicleVisualSystemIrrlicht
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator - 4-Patch Terrain with Heightmap and Bump")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # follow vehicle
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                            # directional light
vis.AttachVehicle(gator.GetVehicle())

# Interactive driver
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0       # seconds to full steering deflection
throttle_time = 1.0       # seconds to full throttle
braking_time = 0.3        # seconds to full brake
driver.SetSteeringDelta(1.0 / 50.0 / steering_time)
driver.SetThrottleDelta(1.0 / 50.0 / throttle_time)
driver.SetBrakingDelta(1.0 / 50.0 / braking_time)
driver.Initialize()

step_size = 1e-3           # physics time step
sim_end = 60.0             # run for 60 s to allow exploration of all patches
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))        # render cadence


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
while vis.Run() and gator.GetSystem().GetChTime() < sim_end:
    time = gator.GetSystem().GetChTime()

    if step_number % render_every == 0:                             # throttled rendering
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
    realtime_timer.Spin(step_size)
