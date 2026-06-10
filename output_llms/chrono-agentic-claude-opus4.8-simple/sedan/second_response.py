import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                             # first sedan spawn location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # QUNIT, no initial heading
init_loc2 = chrono.ChVector3d(6, -6, 0.5)                           # second sedan spawn location
init_rot2 = chrono.QuatFromAngleZ(chrono.CH_PI / 2.0)              # second sedan rotated 90 deg about Z

step_size = 1e-3                                                    # integration step
tire_step_size = step_size                                         # tire substep
render_step_size = 1.0 / 50.0                                       # 50 fps render cadence

vehicle = veh.BMW_E90()                                            # first BMW E90 sedan (owns the system)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)               # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)            # no chassis collision shape
vehicle.SetChassisFixed(False)                                     # MANDATORY — chassis must move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))    # initial pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                      # TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                            # tire integration step
vehicle.Initialize()                                               # build the first vehicle

system = vehicle.GetSystem()                                       # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())            # report first vehicle mass

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)    # mesh chassis
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH) # mesh suspension
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)   # mesh steering
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)      # mesh wheels
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)       # mesh tires

vehicle2 = veh.BMW_E90(vehicle.GetSystem())                        # second sedan SHARES the first vehicle's system
vehicle2.SetChassisCollisionType(veh.CollisionType_NONE)           # no chassis collision shape
vehicle2.SetChassisFixed(False)                                    # MANDATORY — chassis must move
vehicle2.SetInitPosition(chrono.ChCoordsysd(init_loc2, init_rot2)) # initial pose of second sedan
vehicle2.SetTireType(veh.TireModelType_TMEASY)                     # TMEASY tire model
vehicle2.SetTireStepSize(tire_step_size)                           # tire integration step
vehicle2.Initialize()                                              # build the second vehicle on the shared system
print("VEHICLE MASS: ", vehicle2.GetVehicle().GetMass())           # report second vehicle mass

vehicle2.SetChassisVisualizationType(veh.VisualizationType_MESH)   # mesh chassis
vehicle2.SetSuspensionVisualizationType(veh.VisualizationType_MESH)# mesh suspension
vehicle2.SetSteeringVisualizationType(veh.VisualizationType_MESH)  # mesh steering
vehicle2.SetWheelVisualizationType(veh.VisualizationType_MESH)     # mesh wheels
vehicle2.SetTireVisualizationType(veh.VisualizationType_MESH)      # mesh tires

terrain = veh.RigidTerrain(system)                                 # rigid terrain shared by both sedans
patch_mat = chrono.ChContactMaterialNSC()                          # NSC patch material
patch_mat.SetFriction(0.9)                                         # tire-ground friction
patch_mat.SetRestitution(0.01)                                     # almost no bounce
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0) # 100x100 flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)  # concrete road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # patch base color
terrain.Initialize()                                               # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-specific Irrlicht visual system
vis.SetWindowTitle('Sedan')                                        # window title
vis.SetWindowSize(1280, 1024)                                      # window resolution
vis.SetChaseCamera(chrono.ChVector3d(-5, 0, 1.8), 6.0, 0.5)        # chase camera trackpoint/dist/height
vis.Initialize()                                                   # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo overlay
vis.AddSkyBox()                                                    # skybox
vis.AddLightDirectional()                                          # directional lighting
vis.AttachVehicle(vehicle.GetVehicle())                            # bind first vehicle assets

driver = veh.ChInteractiveDriverIRR(vis)                           # driver for the first sedan
steering_time = 1.0                                                # s to reach full steering
throttle_time = 1.0                                                # s to reach full throttle
braking_time = 0.3                                                 # s to reach full braking
driver.SetSteeringDelta(render_step_size / steering_time)          # steering increment per render step
driver.SetThrottleDelta(render_step_size / throttle_time)          # throttle increment per render step
driver.SetBrakingDelta(render_step_size / braking_time)            # braking increment per render step
driver.Initialize()                                               # build the first driver

driver2 = veh.ChDriver(vehicle2.GetVehicle())                     # driver for the second sedan
driver2.Initialize()                                              # build the second driver

render_steps = math.ceil(render_step_size / step_size)            # physics steps between renders
realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
step_number = 0                                                   # step counter


while vis.Run():                                                  # SCORED CORE — plain truth-form loop
    time = system.GetChTime()                                     # current sim time

    if step_number % render_steps == 0:                           # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver.SetSteering(0.4 * math.sin(0.8 * time))                # sinusoidal steering, first sedan
    driver.SetThrottle(0.3)                                       # constant throttle, first sedan
    driver2.SetSteering(-0.4 * math.sin(0.8 * time))             # opposite sinusoidal steering, second sedan
    driver2.SetThrottle(0.3)                                      # constant throttle, second sedan

    driver_inputs = driver.GetInputs()                            # first vehicle driver command
    driver_inputs2 = driver2.GetInputs()                          # second vehicle driver command

    driver.Synchronize(time)                                      # update first driver
    driver2.Synchronize(time)                                     # update second driver
    terrain.Synchronize(time)                                     # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)             # update first vehicle
    vehicle2.Synchronize(time, driver_inputs2, terrain)           # update second vehicle
    vis.Synchronize(time, driver_inputs)                          # update visualization

    driver.Advance(step_size)                                     # advance first driver
    driver2.Advance(step_size)                                    # advance second driver
    terrain.Advance(step_size)                                    # advance terrain
    vehicle.Advance(step_size)                                    # advance shared system (first vehicle)
    vehicle2.Advance(step_size)                                   # advance second vehicle subsystems
    vis.Advance(step_size)                                        # advance visualization

    step_number += 1                                              # next step
    realtime_timer.Spin(step_size)                                # pace to wall clock
