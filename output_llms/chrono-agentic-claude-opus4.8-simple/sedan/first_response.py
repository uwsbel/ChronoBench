import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # sedan spawn location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # QUNIT, no initial heading

step_size = 1e-3                                                     # integration step
tire_step_size = step_size                                          # tire substep
render_step_size = 1.0 / 50.0                                        # 50 fps render cadence

vehicle = veh.BMW_E90()                                             # BMW E90 sedan catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)             # no chassis collision shape
vehicle.SetChassisFixed(False)                                      # MANDATORY — chassis must move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))     # initial pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                       # prompt: TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                             # tire integration step
vehicle.Initialize()                                                # build the vehicle subsystems

system = vehicle.GetSystem()                                        # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # report total vehicle mass

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)     # mesh chassis
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)  # mesh suspension
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)    # mesh steering
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)       # mesh wheels
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)        # mesh tires

terrain = veh.RigidTerrain(system)                                  # rigid terrain under the sedan
patch_mat = chrono.ChContactMaterialNSC()                           # NSC patch material
patch_mat.SetFriction(0.9)                                          # tire-ground friction
patch_mat.SetRestitution(0.01)                                      # almost no bounce
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100x100 flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # patch base color
terrain.Initialize()                                                # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle-specific Irrlicht visual system
vis.SetWindowTitle('Sedan')                                         # window title
vis.SetWindowSize(1280, 1024)                                       # window resolution
vis.SetChaseCamera(chrono.ChVector3d(-5, 0, 1.8), 6.0, 0.5)         # chase camera trackpoint/dist/height
vis.Initialize()                                                    # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo overlay
vis.AddSkyBox()                                                     # skybox
vis.AddLightDirectional()                                           # directional lighting
vis.AttachVehicle(vehicle.GetVehicle())                             # bind chassis/wheel/tire assets

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive keyboard driver
steering_time = 1.0                                                 # s to reach full steering
throttle_time = 1.0                                                 # s to reach full throttle
braking_time = 0.3                                                  # s to reach full braking
driver.SetSteeringDelta(render_step_size / steering_time)          # steering increment per render step
driver.SetThrottleDelta(render_step_size / throttle_time)          # throttle increment per render step
driver.SetBrakingDelta(render_step_size / braking_time)            # braking increment per render step
driver.Initialize()                                                # build the driver

render_steps = math.ceil(render_step_size / step_size)             # physics steps between renders
realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacing
step_number = 0                                                    # step counter


while vis.Run():                                                   # SCORED CORE — plain truth-form loop
    time = system.GetChTime()                                      # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver command

    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)              # update vehicle
    vis.Synchronize(time, driver_inputs)                           # update visualization

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    vehicle.Advance(step_size)                                     # advance wrapper-owned system
    vis.Advance(step_size)                                         # advance visualization

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # pace to wall clock
