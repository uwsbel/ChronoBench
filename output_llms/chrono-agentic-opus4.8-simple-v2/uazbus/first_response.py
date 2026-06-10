import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

initLoc = chrono.ChVector3d(0, 0, 0.4)                               # vehicle spawn location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                           # vehicle spawn orientation (identity)

step_size = 1e-3                                                      # integration time step (s)
tire_step_size = step_size                                           # tire model sub-step
render_step_size = 1.0 / 50.0                                        # render cadence (50 fps)

contact_method = chrono.ChContactMethod_NSC                          # rigid terrain -> NSC contacts

terrainHeight = 0                                                     # flat terrain height
terrainLength = 100.0                                                 # terrain size in X (m)
terrainWidth = 100.0                                                  # terrain size in Y (m)

vehicle = veh.UAZBUS()                                                # UAZBUS catalog wrapper
vehicle.SetContactMethod(contact_method)                             # NSC for the rigid road
vehicle.SetChassisFixed(False)                                       # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))        # initial pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire force model
vehicle.SetTireStepSize(tire_step_size)                              # tire sub-step
vehicle.Initialize()                                                 # build the vehicle subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)      # chassis mesh
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)        # wheels mesh
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)         # tires mesh

system = vehicle.GetSystem()                                         # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid terrain on the shared system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material
patch_mat.SetFriction(0.9)                                           # terrain friction
patch_mat.SetRestitution(0.01)                                       # terrain restitution
patch = terrain.AddPatch(patch_mat,                                  # flat patch at origin
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                        # patch tint
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # patch texture
terrain.Initialize()                                                 # build terrain collision/visual

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle Irrlicht visual system
vis.SetWindowTitle('UAZBUS Demo')                                    # window title
vis.SetWindowSize(1280, 1024)                                        # window size (px)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)      # chase cam: point, dist, height
vis.Initialize()                                                     # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo (after Initialize)
vis.AddLightDirectional()                                            # directional light (vehicle truths)
vis.AddSkyBox()                                                      # sky box
vis.AttachVehicle(vehicle.GetVehicle())                              # bind chassis/wheel/tire assets

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive driver bound to vis
steering_time = 1.0                                                   # s to ramp steering 0 -> 1
throttle_time = 1.0                                                   # s to ramp throttle 0 -> 1
braking_time = 0.3                                                    # s to ramp brake 0 -> 1
driver.SetSteeringDelta(render_step_size / steering_time)            # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)            # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)              # braking rate
driver.Initialize()                                                  # build the driver

render_steps = math.ceil(render_step_size / step_size)              # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
step_number = 0                                                      # physics step counter
sim_end = 12.0                                                        # simulation duration (s)

render_every = max(1, render_steps)                                  # untagged cadence constant

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                       # current sim time

    if step_number % render_steps == 0:                             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                              # current driver command

    driver.Synchronize(time)                                        # update driver
    terrain.Synchronize(time)                                       # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)              # update vehicle from inputs+terrain
    vis.Synchronize(time, driver_inputs)                           # update visual HUD


    driver.Advance(step_size)                                       # advance driver
    terrain.Advance(step_size)                                      # advance terrain
    vehicle.Advance(step_size)                                      # advances the wrapper-owned system
    vis.Advance(step_size)                                          # advance visuals

    step_number += 1                                                # next step
    realtime_timer.Spin(step_size)                                  # spin so wall-clock matches sim time
