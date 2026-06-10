import math                                                            # render-cadence math
import os                                                              # env flag for record mode
import pychrono.core as chrono                                         # core PyChrono types
import pychrono.vehicle as veh                                         # vehicle catalog + terrain
import pychrono.irrlicht as chronoirr                                  # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                               # chassis spawn above ground
init_rot = chrono.QuatFromAngleZ(0)                                  # no initial heading rotation
step_size = 1e-3                                                      # integration step (s)
tire_step_size = 1e-3                                                 # tire model sub-step (s)

vehicle = veh.MAN_10t()                                               # MAN 10t catalog truck
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)             # no chassis collision shape
vehicle.SetChassisFixed(False)                                       # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))     # initial pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                       # prompt: TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                             # tire integration step
vehicle.Initialize()                                                 # build the vehicle subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)  # chassis as mesh
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)    # wheels as mesh
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)     # tires as mesh

system = vehicle.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                    # rigid terrain on the same system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                            # tire-ground friction
patch_mat.SetRestitution(0.01)                                        # nearly inelastic ground
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # flat 200x200 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # customizable terrain texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # ground tint
terrain.Initialize()                                                  # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht system
vis.SetWindowTitle("MAN 10t Truck on Rigid Terrain")               # window title
vis.SetWindowSize(1280, 1024)                                        # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera (track point, dist, height)
vis.Initialize()                                                      # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # customizable logo
vis.AddSkyBox()                                                       # sky box backdrop
vis.AddLightDirectional()                                            # directional lighting (vehicle truth)
vis.AttachVehicle(vehicle.GetVehicle())                             # bind chassis/wheel/tire visuals

render_step_size = 1.0 / 50.0                                        # render at 50 fps
render_steps = math.ceil(render_step_size / step_size)             # physics steps per rendered frame

driver = veh.ChInteractiveDriverIRR(vis)                            # real-time steering/throttle/braking
steering_time = 1.0                                                  # s to ramp steering 0 -> 1
throttle_time = 1.0                                                  # s to ramp throttle 0 -> 1
braking_time = 0.3                                                   # s to ramp braking 0 -> 1
driver.SetSteeringDelta(render_step_size / steering_time)          # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)          # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)            # braking rate
driver.Initialize()                                                  # finalize driver


realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacing
step_number = 0                                                     # physics step counter
while vis.Run():
    time = system.GetChTime()                                      # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver command


    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)            # feed inputs to the vehicle
    vis.Synchronize(time, driver_inputs)                          # update HUD/view

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    vehicle.Advance(step_size)                                     # advances the wrapper-owned system
    vis.Advance(step_size)                                         # advance visualization


    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                # spin so wall-clock matches sim time
