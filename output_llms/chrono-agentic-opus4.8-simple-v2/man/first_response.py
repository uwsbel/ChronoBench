import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # spawn point above the road
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # facing +X, no rotation
step_size = 1e-3                                                     # integration step
tire_step_size = 1e-3                                                # tire substep
sim_end = 12.0                                                       # simulation duration (s)

vehicle = veh.MAN_10t()                                              # MAN 10t catalog truck
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)             # no chassis collision shape
vehicle.SetChassisFixed(False)                                      # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))    # initial pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                       # prompt: TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                            # tire integration substep
vehicle.Initialize()                                                # build the vehicle subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)  # mesh chassis body
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension links
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering links
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)    # wheel rims
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)     # tire meshes

system = vehicle.GetSystem()                                        # wrapper owns the system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED contact scene
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())            # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # rigid flat terrain
patch_mat = chrono.ChContactMaterialNSC()                          # NSC patch material
patch_mat.SetFriction(0.9)                                          # tire/road friction
patch_mat.SetRestitution(0.01)                                      # near-inelastic contact
terrainLength = 200.0                                               # X size of the patch
terrainWidth = 200.0                                                # Y size of the patch
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # customizable road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                      # patch tint
terrain.Initialize()                                                # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle-specific Irrlicht system
vis.SetWindowTitle("MAN 10t truck on rigid terrain")               # window title
vis.SetWindowSize(1280, 1024)                                       # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera behind the truck
vis.Initialize()                                                    # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # customizable logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # directional lighting (vehicle demos)
vis.AttachVehicle(vehicle.GetVehicle())                            # bind chassis/wheel/tire assets

render_step_size = 1.0 / 50.0                                       # render cadence (50 fps)
render_steps = math.ceil(render_step_size / step_size)             # physics steps per frame

driver = veh.ChInteractiveDriverIRR(vis)                            # real-time interactive driver
steering_time = 1.0                                                 # 0 -> 1 steering in 1 s
throttle_time = 1.0                                                 # 0 -> 1 throttle in 1 s
braking_time = 0.3                                                  # 0 -> 1 braking in 0.3 s
driver.SetSteeringDelta(render_step_size / steering_time)          # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)          # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)            # braking ramp rate
driver.Initialize()                                                # finalize the driver

render_every = max(1, round(1.0 / (50.0 * step_size)))             # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacing
step_number = 0                                                    # physics step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                      # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current steering/throttle/brake

    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)             # feed inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                          # update HUD/visuals

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    vehicle.Advance(step_size)                                     # advances the wrapper-owned system
    vis.Advance(step_size)                                         # advance visuals

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # spin to match wall-clock
