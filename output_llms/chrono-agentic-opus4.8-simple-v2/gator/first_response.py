import math                                                            # render-cadence ceil
import pychrono.core as chrono                                         # core API
import pychrono.vehicle as veh                                         # vehicle catalog wrappers
import pychrono.irrlicht as chronoirr                                  # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                                # vehicle spawn location (Z above terrain)
init_rot = chrono.QuatFromAngleZ(0)                                    # facing +X, no yaw
step_size = 1e-3                                                       # dynamics time step
tire_step_size = 1e-3                                                  # tire model sub-step

# --- Gator vehicle (catalog wrapper owns its own ChSystem) ---
vehicle = veh.Gator()                                                  # Gator catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision mesh
vehicle.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                        # prompt: TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                              # tire integration step
vehicle.Initialize()                                                  # build the vehicle subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)      # mesh visualization for all components
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

system = vehicle.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # truth's literal banner

# --- Rigid flat terrain with custom texture ---
terrainLength = 100.0                                                 # X size of terrain (m)
terrainWidth = 100.0                                                  # Y size of terrain (m)
terrain = veh.RigidTerrain(system)                                    # rigid terrain on the vehicle's system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material (pairs with NSC method)
patch_mat.SetFriction(0.9)                                            # ground friction coefficient
patch_mat.SetRestitution(0.01)                                        # near-zero bounciness
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM,                 # flat patch centered at origin
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"),      # custom tiled texture
                 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                        # patch tint
terrain.Initialize()                                                  # finalize terrain bodies

# --- Vehicle-specific Irrlicht visualization ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # wheeled-vehicle visual system
vis.SetWindowTitle("Gator on Rigid Terrain")                        # window title
vis.SetWindowSize(1280, 1024)                                        # window pixel size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)         # chase point, distance, height offset
vis.Initialize()                                                     # create the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo
vis.AddSkyBox()                                                       # sky box
vis.AddLightDirectional()                                            # vehicle demos use a directional light
vis.AttachVehicle(vehicle.GetVehicle())                              # bind chassis/wheel/tire visual assets

# --- Interactive driver (keyboard steering / throttle / braking) ---
render_step_size = 1.0 / 50.0                                         # 50 fps real-time render cadence
driver = veh.ChInteractiveDriverIRR(vis)                             # interactive driver bound to the vis
steering_time = 1.0                                                   # s to go 0 -> +1 steering
throttle_time = 1.0                                                   # s to go 0 -> +1 throttle
braking_time = 0.3                                                    # s to go 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)           # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)           # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)             # braking ramp rate
driver.Initialize()                                                  # finalize driver

render_steps = math.ceil(render_step_size / step_size)              # physics steps per rendered frame
render_every = render_steps                                          # untagged cadence constant
realtime_timer = chrono.ChRealtimeStepTimer()                        # wall-clock pacing
step_number = 0                                                       # physics step counter


while vis.Run():                                                      # real-time interactive loop
    time = system.GetChTime()                                        # current sim time

    if step_number % render_steps == 0:                              # throttled rendering (50 fps)
        vis.BeginScene()                                             # start frame
        vis.Render()                                                 # draw scene
        vis.EndScene()                                               # finish frame

    driver_inputs = driver.GetInputs()                              # latest steering/throttle/brake

    driver.Synchronize(time)                                         # update driver
    terrain.Synchronize(time)                                        # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)               # feed inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                            # update visualization HUD

    driver.Advance(step_size)                                        # advance driver
    terrain.Advance(step_size)                                       # advance terrain
    vehicle.Advance(step_size)                                       # advances the wrapper-owned system
    vis.Advance(step_size)                                           # advance visualization


    step_number += 1                                                 # count the physics step
    realtime_timer.Spin(step_size)                                  # pace to wall clock
