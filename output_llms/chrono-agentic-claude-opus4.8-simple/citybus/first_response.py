import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # spawn position on the rigid terrain
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # QUNIT, no initial rotation

step_size = 1e-3                                                     # integration step
tire_step_size = step_size                                          # tire force model step
render_step_size = 1.0 / 50.0                                       # 50 fps render cadence

bus = veh.CityBus()                                                 # catalog city bus wrapper (owns its ChSystem)
bus.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)                 # no chassis collision shape
bus.SetChassisFixed(False)                                          # chassis must be free to move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # initial location + orientation
bus.SetTireType(veh.TireModelType_TMEASY)                           # TMeasy tire model
bus.SetTireStepSize(tire_step_size)                                 # tire integration step
bus.Initialize()                                                    # build the vehicle subsystems

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)         # mesh body for the chassis
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # primitive suspension links
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)  # primitive steering links
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)           # mesh wheels
bus.SetTireVisualizationType(veh.VisualizationType_MESH)            # mesh tires

system = bus.GetSystem()                                            # the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())                # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # flat rigid terrain
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material
patch_mat.SetFriction(0.9)                                          # tire-road friction
patch_mat.SetRestitution(0.01)                                      # near-inelastic contacts
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100x100 patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # custom tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # terrain tint
terrain.Initialize()                                               # finalize the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle-specific Irrlicht vis
vis.SetWindowTitle('City Bus Demo')                                 # window title
vis.SetWindowSize(1280, 1024)                                       # window resolution
vis.SetChaseCamera(chrono.ChVector3d(-15, 10, 5.8), 6.0, 3.5)      # follow camera (trackPoint, dist, height)
vis.Initialize()                                                   # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # corner logo
vis.AddSkyBox()                                                    # sky backdrop
vis.AddLightDirectional()                                          # directional scene light
vis.AttachVehicle(bus.GetVehicle())                                # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                           # keyboard steering/throttle/braking
driver.SetSteeringDelta(render_step_size / 1.0)                    # steering ramp rate
driver.SetThrottleDelta(render_step_size / 1.0)                    # throttle ramp rate
driver.SetBrakingDelta(render_step_size / 0.3)                     # braking ramp rate
driver.Initialize()                                                # finalize the driver

render_steps = math.ceil(render_step_size / step_size)             # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacing
step_number = 0                                                    # step counter for render throttling


while vis.Run():
    time = system.GetChTime()                                      # current sim time

    if step_number % render_steps == 0:                            # throttled rendering at 50 fps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current steering/throttle/braking

    driver.Synchronize(time)                                       # update driver state
    terrain.Synchronize(time)                                      # update terrain
    bus.Synchronize(time, driver_inputs, terrain)                  # feed inputs + terrain to the vehicle
    vis.Synchronize(time, driver_inputs)                           # update the visual system

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    bus.Advance(step_size)                                         # advance the wrapper-owned system
    vis.Advance(step_size)                                         # advance the visual system

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # pace to wall clock
