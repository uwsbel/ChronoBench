import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle assets

step_size = 5e-3                                                    # physics timestep (s)
sim_end = 120.0                                                     # max simulation time (s)
render_fps = 50.0                                                   # rendering frequency (Hz)
render_every = max(1, round(1.0 / (render_fps * step_size)))        # steps per render frame

# --- Vehicle setup ---
hmmwv = veh.HMMWV_Full()                                           # full HMMWV model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision
hmmwv.SetChassisFixed(False)                                        # allow chassis to move

init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)                        # spawn above flat patch
init_rot = chrono.QuatFromAngleZ(0.0)                               # no initial yaw
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

hmmwv.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY for rigid terrain
hmmwv.SetTireStepSize(step_size)

hmmwv.Initialize()
system = hmmwv.GetSystem()                                          # take system from wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # scored core vehicle mass

# Mesh visualization for all vehicle components
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)       # mesh for chassis
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # primitives for suspension
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # primitives for steering
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)         # mesh for wheels
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)          # mesh for tires

# --- Multi-patch rigid terrain ---
terrain = veh.RigidTerrain(system)

# Patch 1: flat center patch with tile texture
patch_mat1 = chrono.ChContactMaterialNSC()
patch_mat1.SetFriction(0.9)
patch_mat1.SetRestitution(0.01)
patch1 = terrain.AddPatch(
    patch_mat1,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),  # centered at origin
    200.0, 200.0                                                     # 200x200 m flat patch
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # yellowish tint

# Patch 2: flat side patch with different (grass) texture
patch_mat2 = chrono.ChContactMaterialNSC()
patch_mat2.SetFriction(0.8)
patch_mat2.SetRestitution(0.01)
patch2 = terrain.AddPatch(
    patch_mat2,
    chrono.ChCoordsysd(chrono.ChVector3d(250.0, 0, 0), chrono.QUNIT),
    100.0, 200.0                                                     # secondary flat patch
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 100, 200) # grass texture
patch2.SetColor(chrono.ChColor(0.4, 0.8, 0.4))                     # greenish tint

# Patch 3: mesh-based bump patch
patch_mat3 = chrono.ChContactMaterialNSC()
patch_mat3.SetFriction(0.9)
patch_mat3.SetRestitution(0.01)
patch3 = terrain.AddPatch(
    patch_mat3,
    chrono.ChCoordsysd(chrono.ChVector3d(-100.0, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj")                      # bump mesh patch
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)   # tile texture on bump
patch3.SetColor(chrono.ChColor(0.6, 0.6, 0.6))                     # grey

# Patch 4: heightmap patch for varying elevations
patch_mat4 = chrono.ChContactMaterialNSC()
patch_mat4.SetFriction(0.9)
patch_mat4.SetRestitution(0.01)
patch4 = terrain.AddPatch(
    patch_mat4,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 250.0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),               # heightmap BMP
    64.0, 64.0,                                                      # patch size (m)
    0.0, 3.0                                                         # height min/max (m)
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 64, 64)   # textured heightmap
patch4.SetColor(chrono.ChColor(0.7, 0.7, 0.5))                     # terrain color

terrain.Initialize()

# --- Irrlicht vehicle visualization (Initialize FIRST, scene elements AFTER) ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Multi-Patch Rigid Terrain")               # window title
vis.SetWindowSize(1280, 720)                                        # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)   # chase camera setup
vis.Initialize()                                                    # MUST be before AddSkyBox/lights
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # project logo
vis.AddSkyBox()                                                     # sky background
vis.AddLightDirectional()                                           # directional light
vis.AttachVehicle(hmmwv.GetVehicle())                               # bind vehicle visuals

# --- Interactive driver (keyboard control) ---
driver = veh.ChInteractiveDriverIRR(vis)                            # keyboard interactive driver
steering_time = 1.0                                                 # s to reach full steering
throttle_time = 1.0                                                 # s to reach full throttle
braking_time = 0.3                                                  # s to reach full braking
driver.SetSteeringDelta(render_every * step_size / steering_time)   # per render step delta
driver.SetThrottleDelta(render_every * step_size / throttle_time)
driver.SetBrakingDelta(render_every * step_size / braking_time)
driver.Initialize()

# --- Review-only recording setup ---


realtime_timer = chrono.ChRealtimeStepTimer()                       # real-time pacing
step_number = 0                                                     # step counter for render throttle

# --- Simulation loop ---
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                       # current sim time (s)

    if step_number % render_every == 0:                             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                              # get current inputs


    driver.Synchronize(time)                                        # update driver state
    terrain.Synchronize(time)                                       # update terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)                 # update vehicle
    vis.Synchronize(time, driver_inputs)                            # update visualization

    driver.Advance(step_size)                                       # advance driver
    terrain.Advance(step_size)                                      # advance terrain
    hmmwv.Advance(step_size)                                        # advance vehicle + system
    vis.Advance(step_size)                                          # advance visualization

    step_number += 1
    realtime_timer.Spin(step_size)                                  # real-time pacing
