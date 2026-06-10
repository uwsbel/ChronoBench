import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                                # vehicle spawn location
init_rot = chrono.QuatFromAngleZ(0.0)                                  # vehicle heading (no yaw)
step_size = 1e-3                                                       # dynamics time step
tire_step_size = 1e-3                                                  # tire force model sub-step
render_step_size = 1.0 / 50.0                                          # 50 fps render cadence

bus = veh.CityBus()                                                    # catalog CityBus wrapper (owns its system)
bus.SetContactMethod(chrono.ChContactMethod_NSC)                      # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)                  # no chassis collision geometry
bus.SetChassisFixed(False)                                            # chassis must be free to move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))          # initial pose
bus.SetTireType(veh.TireModelType_TMEASY)                            # TMEASY tire force model
bus.SetTireStepSize(tire_step_size)                                  # tire integration step
bus.Initialize()                                                      # build the vehicle subsystems

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)         # chassis drawn as mesh
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)   # suspension as primitives
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)     # steering as primitives
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)           # wheels drawn as mesh
bus.SetTireVisualizationType(veh.VisualizationType_MESH)            # tires drawn as mesh

system = bus.GetSystem()                                              # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())                  # report total vehicle mass

terrain = veh.RigidTerrain(system)                                    # flat rigid ground
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                            # tire-ground friction
patch_mat.SetRestitution(0.01)                                        # near-inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)   # 200 x 200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)   # custom road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                        # patch tint
terrain.Initialize()                                                  # finalize the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-aware Irrlicht visual system
vis.SetWindowTitle("CityBus on Rigid Terrain")                       # window title
vis.SetWindowSize(1280, 1024)                                        # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 14.0, 0.5)    # follow camera (track point, dist, height)
vis.Initialize()                                                      # build the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # corner logo
vis.AddSkyBox()                                                       # sky box backdrop
vis.AddLightDirectional()                                            # single directional light (vehicle convention)
vis.AttachVehicle(bus.GetVehicle())                                  # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive keyboard driver
steering_time = 1.0                                                   # s to reach full steering
throttle_time = 1.0                                                   # s to reach full throttle
braking_time = 0.3                                                    # s to reach full brake
driver.SetSteeringDelta(render_step_size / steering_time)           # per-frame steering increment
driver.SetThrottleDelta(render_step_size / throttle_time)           # per-frame throttle increment
driver.SetBrakingDelta(render_step_size / braking_time)             # per-frame brake increment
driver.Initialize()                                                  # finalize the driver

render_steps = math.ceil(render_step_size / step_size)              # physics steps between frames
sim_end = 12.0                                                        # simulation horizon (s)
realtime_timer = chrono.ChRealtimeStepTimer()                       # spin to wall-clock real time
step_number = 0                                                      # physics step counter

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                        # current sim time

    if step_number % render_steps == 0:                             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                              # current steering/throttle/brake

    driver.Synchronize(time)                                        # update driver
    terrain.Synchronize(time)                                       # update terrain
    bus.Synchronize(time, driver_inputs, terrain)                  # feed inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                           # update HUD/visuals

    driver.Advance(step_size)                                       # advance driver
    terrain.Advance(step_size)                                      # advance terrain
    bus.Advance(step_size)                                          # advances the wrapper-owned system
    vis.Advance(step_size)                                          # advance visuals


    step_number += 1                                               # advance step counter
    realtime_timer.Spin(step_size)                                 # match wall-clock to sim time
