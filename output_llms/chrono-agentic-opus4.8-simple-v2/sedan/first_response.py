import math                                                           # render-cadence math
import pychrono.core as chrono                                        # Chrono core
import pychrono.vehicle as veh                                        # vehicle catalog + terrain + driver
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate bundled vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn (origin, above the road)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation
step_size = 2e-3                                                      # integration step (s)
tire_step_size = 1e-3                                                 # tire force-model substep (s)

vehicle = veh.BMW_E90()                                              # BMW E90 sedan catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)             # no chassis collision against the road
vehicle.SetChassisFixed(False)                                       # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial chassis pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                        # prompt: TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                              # tire substep size
vehicle.Initialize()                                                 # build the vehicle subsystem stack

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)      # configurable visualization — chassis mesh
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)        # wheel mesh
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)         # tire mesh

system = vehicle.GetSystem()                                         # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED — Bullet collision, after Initialize
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # truth's literal mass banner

terrain = veh.RigidTerrain(system)                                   # rigid terrain on the vehicle's system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for the road
patch_mat.SetFriction(0.9)                                           # road friction coefficient
patch_mat.SetRestitution(0.01)                                       # near-inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)   # flat 200x200 m patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # customizable road texture, UV-tiled
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                        # road surface tint
terrain.Initialize()                                                 # build the terrain collision/visual

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht visual system
vis.SetWindowTitle("BMW E90 Sedan on Rigid Terrain")                # window title (before Initialize)
vis.SetWindowSize(1280, 1024)                                        # window resolution (before Initialize)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)     # chase camera: track point, distance, height
vis.Initialize()                                                     # create the Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # customizable logo overlay (after Initialize)
vis.AddSkyBox()                                                      # sky box backdrop (after Initialize)
vis.AddLightDirectional()                                           # directional lighting (vehicle-demo standard)
vis.AttachVehicle(vehicle.GetVehicle())                             # bind chassis/wheel/tire visual assets

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver bound to the visual system
steering_time = 1.0                                                 # s to ramp steering 0 -> +1
throttle_time = 1.0                                                 # s to ramp throttle 0 -> +1
braking_time = 0.3                                                  # s to ramp braking 0 -> +1
render_step_size = 1.0 / 50.0                                       # render cadence period (s)
driver.SetSteeringDelta(render_step_size / steering_time)          # per-render steering increment
driver.SetThrottleDelta(render_step_size / throttle_time)          # per-render throttle increment
driver.SetBrakingDelta(render_step_size / braking_time)            # per-render braking increment
driver.Initialize()                                                # arm the interactive driver

render_steps = math.ceil(render_step_size / step_size)             # untagged: physics steps per rendered frame
sim_end = 20.0                                                      # simulation duration (s)
realtime_timer = chrono.ChRealtimeStepTimer()                      # spin to keep wall-clock = sim time
step_number = 0                                                     # physics step counter
while vis.Run() and system.GetChTime() < sim_end:                  # real-time render loop
    time = system.GetChTime()                                      # current sim time

    if step_number % render_steps == 0:                            # throttled rendering once per frame
        vis.BeginScene()                                           # open the render frame
        vis.Render()                                               # draw the scene
        vis.EndScene()                                             # present the frame

    driver_inputs = driver.GetInputs()                             # current steering/throttle/brake

    driver.Synchronize(time)                                       # feed time to the driver
    terrain.Synchronize(time)                                      # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)             # apply driver inputs against terrain
    vis.Synchronize(time, driver_inputs)                          # update HUD/camera with inputs

    driver.Advance(step_size)                                      # advance the driver
    terrain.Advance(step_size)                                     # advance the terrain
    vehicle.Advance(step_size)                                     # advance the vehicle (steps the system)
    vis.Advance(step_size)                                         # advance the visualization


    step_number += 1                                               # count this physics step
    realtime_timer.Spin(step_size)                                # spin so wall-clock matches sim time
