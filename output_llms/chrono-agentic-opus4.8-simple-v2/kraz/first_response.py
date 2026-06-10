import math                                                               # render-cadence math
import pychrono.core as chrono                                            # core PyChrono
import pychrono.vehicle as veh                                            # vehicle catalog
import pychrono.irrlicht as chronoirr                                     # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                     # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')                 # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                                  # initial chassis location (Z-up)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                              # initial chassis orientation (identity)
init_fwd_vel = 0.0                                                       # initial forward velocity (m/s)

step_size = 2e-3                                                         # integration step (s)
tire_step_size = 1e-3                                                    # tire model substep (s)

vehicle = veh.Kraz()                                                     # Kraz tractor-trailer truck
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)                  # no wrapper chassis collision mesh
vehicle.SetChassisFixed(False)                                          # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # spawn pose
vehicle.SetInitFwdVel(init_fwd_vel)                                     # start from rest
vehicle.SetTireStepSize(tire_step_size)                                 # tire substep
vehicle.Initialize()                                                    # build the tractor-trailer

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)  # mesh chassis (tractor, trailer)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)  # primitive steering
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)  # primitive suspension
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)  # mesh wheels (tractor, trailer)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)  # mesh tires (tractor, trailer)

system = vehicle.GetSystem()                                            # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())                 # report tractor mass

terrain = veh.RigidTerrain(system)                                      # flat rigid terrain
patch_mat = chrono.ChContactMaterialNSC()                               # NSC patch material
patch_mat.SetFriction(0.9)                                              # terrain friction
patch_mat.SetRestitution(0.01)                                          # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 300.0, 300.0)      # 300x300 m flat patch
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                           # sandy color
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
terrain.Initialize()                                                    # build the terrain

chase_track_point = chrono.ChVector3d(0, 0, 1.75)                       # point on tractor to follow
chase_distance = 12.0                                                   # camera distance behind tractor (m)
chase_height = 0.5                                                      # camera height offset (m)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                        # vehicle-specific Irrlicht vis
vis.SetWindowTitle("Kraz Truck Demo")                                   # window title
vis.SetWindowSize(1280, 1024)                                           # window size in pixels
vis.SetChaseCamera(chase_track_point, chase_distance, chase_height)     # chase cam (track point, dist, height)
vis.SetChaseCameraState(veh.ChChaseCamera.Chase)                       # follow-from-behind camera mode
vis.SetChaseCameraPosition(chrono.ChVector3d(-12, 0, 2.0))             # initial camera world position
vis.Initialize()                                                        # create the Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))       # corner logo
vis.AddSkyBox()                                                         # sky box
vis.AddLightDirectional()                                              # directional light (vehicle truths use this)
vis.AttachVehicle(vehicle.GetTractor())                                # bind tractor visual assets

render_step_size = 1.0 / 50.0                                           # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)                  # physics steps per rendered frame

driver = veh.ChInteractiveDriverIRR(vis)                                # interactive (keyboard) driver
steering_time = 1.0                                                     # s to reach full steering
throttle_time = 1.0                                                     # s to reach full throttle
braking_time = 0.3                                                      # s to reach full braking
driver.SetSteeringDelta(render_step_size / steering_time)              # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)              # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)                # braking rate
driver.Initialize()                                                    # initialize the driver

sim_end = 12.0                                                          # simulation end time (s)

realtime_timer = chrono.ChRealtimeStepTimer()                          # wall-clock pacing
step_number = 0                                                        # physics step counter
while vis.Run():                                                       # real-time render loop
    time = system.GetChTime()                                          # current sim time

    if step_number % render_steps == 0:                                # throttled rendering
        vis.BeginScene()                                               # begin frame
        vis.Render()                                                   # draw scene
        vis.EndScene()                                                 # end frame

    driver_inputs = driver.GetInputs()                                 # current driver inputs

    driver.Synchronize(time)                                           # sync driver
    terrain.Synchronize(time)                                          # sync terrain
    vehicle.Synchronize(time, driver_inputs, terrain)                  # sync vehicle (3-arg, wheeled)
    vis.Synchronize(time, driver_inputs)                               # sync vis HUD


    driver.Advance(step_size)                                          # advance driver
    terrain.Advance(step_size)                                         # advance terrain
    vehicle.Advance(step_size)                                         # advance vehicle (steps the system)
    vis.Advance(step_size)                                             # advance vis

    step_number += 1                                                   # next step
    realtime_timer.Spin(step_size)                                     # pace to wall-clock

    if time >= sim_end:                                                # stop at end time
        break
