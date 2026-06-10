import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # vehicle spawn location (world)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # vehicle spawn orientation (identity)

step_size = 2e-3                                                     # dynamics integration step
tire_step_size = 1e-3                                                # tire model sub-step

terrainLength = 100.0                                               # terrain size in X (m)
terrainWidth = 100.0                                                # terrain size in Y (m)

vis_type = veh.VisualizationType_PRIMITIVES                          # primitive visualization for vehicle parts

# --- Vehicle: full HMMWV on rigid terrain ---
hmmwv = veh.HMMWV_Full()                                            # full HMMWV catalog wrapper (owns its system)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision on flat road
hmmwv.SetChassisFixed(False)                                       # MANDATORY — fixed chassis would not move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # prompt: TMEASY tire model
hmmwv.SetTireStepSize(tire_step_size)                              # tire integration step
hmmwv.Initialize()                                                 # build subsystems

hmmwv.SetChassisVisualizationType(vis_type)                        # primitive chassis visuals
hmmwv.SetSuspensionVisualizationType(vis_type)                     # primitive suspension visuals
hmmwv.SetSteeringVisualizationType(vis_type)                       # primitive steering visuals
hmmwv.SetWheelVisualizationType(vis_type)                          # primitive wheel visuals
hmmwv.SetTireVisualizationType(vis_type)                           # primitive tire visuals

system = hmmwv.GetSystem()                                         # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())             # report total vehicle mass

# --- Rigid terrain (flat patch with texture) ---
terrain = veh.RigidTerrain(system)                                # terrain bound to the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                         # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                        # road friction
patch_mat.SetRestitution(0.01)                                    # near-inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch at origin
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # patch base color
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
terrain.Initialize()                                              # finalize terrain

# --- Vehicle-specific Irrlicht visualization ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # wheeled-vehicle visual system
vis.SetWindowTitle("HMMWV on Rigid Terrain")                     # window title
vis.SetWindowSize(1280, 1024)                                    # window size in pixels
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)  # chase camera tracking the chassis
vis.Initialize()                                                 # build the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # corner logo
vis.AddSkyBox()                                                  # sky box
vis.AddLightDirectional()                                       # directional light (vehicle-truth lighting)
vis.AttachVehicle(hmmwv.GetVehicle())                           # bind chassis/wheel/tire visuals

# --- Interactive driver (steering / throttle / braking) ---
render_step_size = 1.0 / 50.0                                    # 50 FPS rendering cadence
driver = veh.ChInteractiveDriverIRR(vis)                        # interactive (keyboard) driver bound to vis
steering_time = 1.0                                             # seconds 0 -> +1 steering
throttle_time = 1.0                                             # seconds 0 -> +1 throttle
braking_time = 0.3                                              # seconds 0 -> +1 braking
driver.SetSteeringDelta(render_step_size / steering_time)       # steering rate per render step
driver.SetThrottleDelta(render_step_size / throttle_time)       # throttle rate per render step
driver.SetBrakingDelta(render_step_size / braking_time)         # braking rate per render step
driver.Initialize()                                            # finalize driver

# --- Simulation loop (real-time, render at 50 FPS) ---
render_steps = math.ceil(render_step_size / step_size)          # physics steps between rendered frames
realtime_timer = chrono.ChRealtimeStepTimer()                   # wall-clock pacing timer
step_number = 0                                                # physics step counter
sim_end = 12.0                                                # simulation duration (s)

while vis.Run():
    time = system.GetChTime()                                # current sim time
    if time > sim_end:                                       # stop after sim_end
        break

    if step_number % render_steps == 0:                     # throttled rendering at 50 FPS
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                      # current driver command

    driver.Synchronize(time)                                # propagate driver inputs
    terrain.Synchronize(time)                               # update terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)         # apply inputs + sample terrain
    vis.Synchronize(time, driver_inputs)                   # update HUD/visuals

    driver.Advance(step_size)                              # advance driver
    terrain.Advance(step_size)                             # advance terrain
    hmmwv.Advance(step_size)                               # advance wrapper-owned system
    vis.Advance(step_size)                                # advance visuals

    step_number += 1                                       # next step
    realtime_timer.Spin(step_size)                         # pace to wall clock
