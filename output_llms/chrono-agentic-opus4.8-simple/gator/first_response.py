import math                                                           # render-cadence math
import pychrono.core as chrono                                        # core PyChrono
import pychrono.vehicle as veh                                        # vehicle module

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # vehicle spawn location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # vehicle spawn orientation (identity)

step_size = 1e-3                                                      # integration step (s)
terrainLength = 100.0                                                 # terrain X size (m)
terrainWidth = 100.0                                                  # terrain Y size (m)

gator = veh.Gator()                                                  # Gator catalog wrapper
gator.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
gator.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # initial pose
gator.SetTireType(veh.TireModelType_TMEASY)                         # prompt: TMEASY tire model
gator.SetTireStepSize(step_size)                                     # tire integration step
gator.Initialize()                                                   # build the vehicle subsystems

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)      # mesh visualization — chassis
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)   # mesh visualization — suspension
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)     # mesh visualization — steering
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)        # mesh visualization — wheels
gator.SetTireVisualizationType(veh.VisualizationType_MESH)         # mesh visualization — tires

system = gator.GetSystem()                                           # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED, after Initialize
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                           # terrain friction
patch_mat.SetRestitution(0.01)                                       # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat ground patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)          # custom terrain texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # terrain tint
terrain.Initialize()                                                 # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht window
vis.SetWindowTitle("Gator on Rigid Terrain")                        # window title
vis.SetWindowSize(1280, 1024)                                       # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)        # chase camera behind the vehicle
vis.Initialize()                                                     # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo after Initialize
vis.AddSkyBox()                                                      # sky box after Initialize
vis.AddLightDirectional()                                           # directional light (vehicle demos)
vis.AttachVehicle(gator.GetVehicle())                              # bind vehicle visual assets

render_step_size = 1.0 / 50.0                                        # 50 frames per second
render_steps = math.ceil(render_step_size / step_size)              # physics steps per render frame

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive steering/throttle/brake driver
steering_time = 1.0                                                  # s to reach full steering
throttle_time = 1.0                                                  # s to reach full throttle
braking_time = 0.3                                                   # s to reach full brake
driver.SetSteeringDelta(render_step_size / steering_time)           # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)           # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)             # braking ramp rate
driver.Initialize()                                                 # finalize driver

render_every = max(1, render_steps)                                  # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                       # real-time pacing
sim_end = 20.0                                                       # simulation duration (s)
while vis.Run() and system.GetChTime() < sim_end:                  # main real-time loop
    vis.BeginScene()                                                # begin render
    vis.Render()                                                    # draw the scene
    vis.EndScene()                                                  # end render
    for _ in range(render_every):                                  # advance physics between frames
        time = system.GetChTime()                                  # current sim time
        driver_inputs = driver.GetInputs()                         # read driver inputs
        driver.Synchronize(time)                                   # sync driver
        terrain.Synchronize(time)                                  # sync terrain
        gator.Synchronize(time, driver_inputs, terrain)           # sync vehicle with inputs + terrain
        vis.Synchronize(time, driver_inputs)                       # sync visualization HUD
        driver.Advance(step_size)                                  # advance driver
        terrain.Advance(step_size)                                 # advance terrain
        gator.Advance(step_size)                                   # advance vehicle (steps the system)
        vis.Advance(step_size)                                     # advance visualization
        realtime_timer.Spin(step_size)                             # spin so wall-clock matches sim time
        if system.GetChTime() >= sim_end:                          # stop at sim_end
            break
