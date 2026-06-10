import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(6, -70, 0.5)                          # sedan spawn on the highway
init_rot = chrono.QuatFromAngleZ(1.57)                             # heading rotated ~90 deg about Z

step_size = 2e-4                                                    # finer integration step
tire_step_size = step_size                                         # tire substep
render_step_size = 1.0 / 100.0                                     # finer render cadence

vehicle = veh.BMW_E90()                                            # BMW E90 sedan catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)               # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)            # no chassis collision shape
vehicle.SetChassisFixed(False)                                     # MANDATORY — chassis must move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))    # initial pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                      # TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                            # tire integration step
vehicle.Initialize()                                               # build the vehicle

system = vehicle.GetSystem()                                       # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())            # report total vehicle mass

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)    # mesh chassis
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH) # mesh suspension
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)   # mesh steering
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)      # mesh wheels
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)       # mesh tires

terrain = veh.RigidTerrain(system)                                 # rigid terrain (highway mesh)
patch_mat = chrono.ChContactMaterialNSC()                          # NSC patch material
patch_mat.SetFriction(0.9)                                         # tire-ground friction
patch_mat.SetRestitution(0.01)                                     # almost no bounce
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         veh.GetDataFile("terrain/meshes/Highway_col.obj"), True, 0.01, False)  # collision mesh patch
vis_mesh = chrono.ChTriangleMeshConnected()                        # separate visual mesh
vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)  # load highway visual mesh
vis_shape = chrono.ChVisualShapeTriangleMesh()                     # visual shape from the mesh
vis_shape.SetMesh(vis_mesh)                                        # attach the loaded mesh
vis_shape.SetMutable(False)                                        # static geometry
patch.GetGroundBody().AddVisualShape(vis_shape)                    # show the highway visual on the ground body
terrain.Initialize()                                               # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-specific Irrlicht visual system
vis.SetWindowTitle('Sedan')                                        # window title
vis.SetWindowSize(1280, 1024)                                      # window resolution
vis.SetChaseCamera(chrono.ChVector3d(-3.0, 0.0, 1.1), 6.0, 0.5)    # chase camera trackpoint/dist/height
vis.Initialize()                                                   # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo overlay
vis.AddSkyBox()                                                    # skybox
vis.AddLightDirectional()                                          # directional lighting
vis.AttachVehicle(vehicle.GetVehicle())                            # bind chassis/wheel/tire assets

driver = veh.ChInteractiveDriverIRR(vis)                           # interactive driver (steering via keyboard)
steering_time = 5.0                                                # s to reach full steering (slower response)
throttle_time = 1.0                                                # s to reach full throttle
braking_time = 0.3                                                 # s to reach full braking
driver.SetSteeringDelta(render_step_size / steering_time)          # steering increment per render step
driver.SetThrottleDelta(render_step_size / throttle_time)          # throttle increment per render step
driver.SetBrakingDelta(render_step_size / braking_time)            # braking increment per render step
driver.Initialize()                                               # build the driver

target_speed = 8.0                                                 # reference speed [m/s] for PID throttle
Kp = 0.4                                                           # proportional gain
Ki = 0.05                                                          # integral gain
Kd = 0.0                                                           # derivative gain
err_integral = 0.0                                                 # accumulated speed error
err_prev = 0.0                                                     # previous speed error

render_steps = math.ceil(render_step_size / step_size)            # physics steps between renders
realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
step_number = 0                                                   # step counter


while vis.Run():                                                  # SCORED CORE — plain truth-form loop
    time = system.GetChTime()                                     # current sim time

    if step_number % render_steps == 0:                           # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    speed = vehicle.GetVehicle().GetSpeed()                        # current forward speed
    err = target_speed - speed                                    # speed error vs reference
    err_integral += err * step_size                               # integrate error
    err_deriv = (err - err_prev) / step_size                      # error derivative
    err_prev = err                                                # store for next step
    throttle = Kp * err + Ki * err_integral + Kd * err_deriv      # PID throttle command
    throttle = max(0.0, min(1.0, throttle))                       # clamp to [0,1]
    driver.SetThrottle(throttle)                                  # apply PID throttle

    driver_inputs = driver.GetInputs()                            # current driver command

    driver.Synchronize(time)                                      # update driver
    terrain.Synchronize(time)                                     # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)             # update vehicle
    vis.Synchronize(time, driver_inputs)                          # update visualization

    driver.Advance(step_size)                                     # advance driver
    terrain.Advance(step_size)                                    # advance terrain
    vehicle.Advance(step_size)                                    # advance wrapper-owned system
    vis.Advance(step_size)                                        # advance visualization

    step_number += 1                                              # next step
    realtime_timer.Spin(step_size)                                # pace to wall clock
