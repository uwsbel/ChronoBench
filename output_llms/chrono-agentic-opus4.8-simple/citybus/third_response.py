import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                    # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # locate vehicle data files

step_size = 5e-4                                                        # solver step (reduced for stability)
tire_step_size = 5e-4                                                   # tire step (reduced for stability)

init_loc = chrono.ChVector3d(0, 0, 0.5)                                 # chassis spawn location
init_rot = chrono.QuatFromAngleZ(0)                                     # spawn heading (facing +X)

bus = veh.CityBus()                                                     # catalog city bus wrapper (owns its system)
bus.SetContactMethod(chrono.ChContactMethod_NSC)                       # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)                   # no chassis collision mesh
bus.SetChassisFixed(False)                                             # MANDATORY: chassis must be free to move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))           # set spawn pose
bus.SetTireType(veh.TireModelType_PAC02)                              # prompt: Pacejka tire (CityBus ships the Pacejka MF model as PAC02)
bus.SetTireStepSize(tire_step_size)                                   # tire integration step
bus.Initialize()                                                       # build the vehicle

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)          # chassis mesh
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # suspension primitives
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # steering primitives
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)            # wheel mesh
bus.SetTireVisualizationType(veh.VisualizationType_MESH)             # tire mesh

system = bus.GetSystem()                                               # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for contact scenes
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())                   # report total vehicle mass

terrain = veh.RigidTerrain(system)                                     # rigid flat terrain
patch_mat = chrono.ChContactMaterialNSC()                             # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                             # tire-ground friction
patch_mat.SetRestitution(0.01)                                         # near-inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)   # 200 x 200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)  # prompt: dirt road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                        # terrain tint
terrain.Initialize()                                                   # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                      # vehicle-specific Irrlicht window
vis.SetWindowTitle("City Bus on Dirt Road")                          # window title
vis.SetWindowSize(1280, 1024)                                        # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 14.0, 0.5)        # follow camera behind the bus
vis.Initialize()                                                       # build device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo overlay
vis.AddSkyBox()                                                        # sky backdrop
vis.AddLightDirectional()                                             # directional scene light
vis.AttachVehicle(bus.GetVehicle())                                  # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive driver bound to the window

render_step_size = 1.0 / 50.0                                         # 50 FPS render cadence
steering_time = 1.0                                                   # s to ramp steering 0 -> 1
throttle_time = 1.0                                                   # s to ramp throttle 0 -> 1
braking_time = 0.3                                                    # s to ramp brake 0 -> 1
driver.SetSteeringDelta(render_step_size / steering_time)            # steering responsiveness
driver.SetThrottleDelta(render_step_size / throttle_time)            # throttle responsiveness
driver.SetBrakingDelta(render_step_size / braking_time)              # brake responsiveness
driver.Initialize()                                                  # finalize the driver

render_steps = math.ceil(render_step_size / step_size)               # untagged render cadence (physics steps / frame)

realtime_timer = chrono.ChRealtimeStepTimer()                       # spin so wall-clock matches sim time
step_number = 0                                                       # physics step counter
while vis.Run():                                                     # main real-time loop
    time = system.GetChTime()                                       # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()                                            # start frame
        vis.Render()                                               # draw the scene
        vis.EndScene()                                             # finish frame

    driver_inputs = driver.GetInputs()                            # current driver command

    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    bus.Synchronize(time, driver_inputs, terrain)                 # feed inputs + terrain to the bus
    vis.Synchronize(time, driver_inputs)                          # update visuals/HUD

    driver.Advance(step_size)                                     # advance driver
    terrain.Advance(step_size)                                    # advance terrain
    bus.Advance(step_size)                                        # advance the wrapper-owned system
    vis.Advance(step_size)                                        # advance visuals

    step_number += 1                                              # next step
    realtime_timer.Spin(step_size)                               # spin in place to match wall-clock
