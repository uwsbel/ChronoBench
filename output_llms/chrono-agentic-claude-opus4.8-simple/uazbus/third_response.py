import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # core Chrono data
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data

initLoc = chrono.ChVector3d(0, 0, 0.5)                               # spawn above terrain
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                          # QUNIT (no rotation)

step_size = 1e-3                                                     # integration step
tire_step_size = step_size                                          # tire substep
render_step_size = 1.0 / 50.0                                        # 50 FPS visualization

terrainLength = 100.0                                               # terrain X size
terrainWidth = 100.0                                                # terrain Y size

vehicle = veh.UAZBUS()                                              # UAZBUS catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                # rigid terrain -> NSC
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)            # no chassis collision
vehicle.SetChassisFixed(False)                                      # chassis is free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))       # initial pose
vehicle.SetTireType(veh.TireModelType_RIGID)                        # RIGID tire model
vehicle.SetTireStepSize(tire_step_size)                             # tire integration step
vehicle.Initialize()                                                # build subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheel mesh
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)        # tire mesh

system = vehicle.GetSystem()                                        # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # Bullet collision

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # diagnostic

terrain = veh.RigidTerrain(system)                                  # rigid ground
patch_mat = chrono.ChContactMaterialNSC()                           # NSC terrain material
patch_mat.SetFriction(0.9)                                          # friction coefficient
patch_mat.SetRestitution(0.01)                                      # restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)          # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # patch color
terrain.Initialize()                                                # finalize terrain

# Fixed box obstacle to test vehicle mobility
box_mat = chrono.ChContactMaterialNSC()                             # box contact material
box_mat.SetFriction(0.9)                                            # friction coefficient
box_mat.SetRestitution(0.01)                                        # restitution
box = chrono.ChBodyEasyBox(0.5, 5, 0.2, 1000, True, True, box_mat)  # 0.5x5x0.2 obstacle
box.SetPos(chrono.ChVector3d(5, 0, 0.1))                            # in front of vehicle
box.SetFixed(True)                                                  # fixed in place
system.Add(box)                                                     # add to system

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle Irrlicht vis
vis.SetWindowTitle('UAZBUS Demo')                                   # window title
vis.SetWindowSize(1280, 1024)                                       # window size
vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)         # chase camera
vis.Initialize()                                                    # build device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # directional light
vis.AttachVehicle(vehicle.GetVehicle())                            # bind vehicle assets

driver = veh.ChInteractiveDriverIRR(vis)                            # driver bound to vis
steering_time = 1.0                                                # 0->1 steering time
throttle_time = 1.0                                                # 0->1 throttle time
braking_time = 0.3                                                 # 0->1 braking time
driver.SetSteeringDelta(render_step_size / steering_time)          # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)          # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)            # braking rate
driver.Initialize()                                                # init driver
driver.SetThrottle(0.5)                                            # constant forward throttle

render_steps = math.ceil(render_step_size / step_size)             # steps per frame

realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacing
step_number = 0                                                    # physics step counter
while vis.Run():                                                   # real-time render loop
    time = system.GetChTime()                                      # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver.SetThrottle(0.5)                                        # maintain forward throttle
    driver_inputs = driver.GetInputs()                             # current driver inputs

    driver.Synchronize(time)                                       # sync driver
    terrain.Synchronize(time)                                      # sync terrain
    vehicle.Synchronize(time, driver_inputs, terrain)              # sync vehicle
    vis.Synchronize(time, driver_inputs)                           # sync visuals

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    vehicle.Advance(step_size)                                     # advance vehicle (steps system)
    vis.Advance(step_size)                                         # advance visuals

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # pace to real time
