import math                                                      # render-cadence math
import pychrono.core as chrono                                   # Chrono core
import pychrono.vehicle as veh                                   # vehicle catalog
import pychrono.irrlicht as chronoirr                            # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())            # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')        # locate vehicle data files

step_size = 1e-3                                                 # integration step (s)
init_loc = chrono.ChVector3d(0, 0, 0.5)                          # chassis spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                      # chassis spawn orientation (identity)

hmmwv = veh.HMMWV_Full()                                         # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)              # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)          # no chassis collision geometry
hmmwv.SetChassisFixed(False)                                    # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))  # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                    # prompt: TMEASY tire model
hmmwv.SetTireStepSize(step_size)                               # tire sub-step
hmmwv.Initialize()                                             # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)      # prompt: primitive visualization
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)   # primitive suspension
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)     # primitive steering
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)        # primitive wheels
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)         # primitive tires

system = hmmwv.GetSystem()                                                   # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)         # REQUIRED, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                       # report total vehicle mass

terrainLength = 100.0                                          # terrain size in X (m)
terrainWidth = 100.0                                           # terrain size in Y (m)
terrain = veh.RigidTerrain(system)                            # rigid flat terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                    # NSC patch material
patch_mat.SetFriction(0.9)                                    # tire-ground friction
patch_mat.SetRestitution(0.01)                               # near-inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)   # flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)           # tiled road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                # patch tint
terrain.Initialize()                                         # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()             # vehicle-aware Irrlicht system
vis.SetWindowTitle("HMMWV Rigid Terrain")                    # window title
vis.SetWindowSize(1280, 1024)                                # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5) # chase camera on chassis
vis.Initialize()                                            # build the Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # corner logo
vis.AddSkyBox()                                             # sky box
vis.AddLightDirectional()                                  # vehicle scenes use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                      # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                   # interactive steering/throttle/braking driver
steering_time = 1.0                                        # s to go 0 -> +1 steering
throttle_time = 1.0                                        # s to go 0 -> +1 throttle
braking_time = 0.3                                         # s to go 0 -> +1 brake
render_step_size = 1.0 / 50.0                              # prompt: 50 frames per second
driver.SetSteeringDelta(render_step_size / steering_time) # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time) # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)   # braking ramp rate
driver.Initialize()                                       # finalize the driver

render_steps = math.ceil(render_step_size / step_size)    # physics steps between rendered frames
render_every = render_steps                               # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()             # wall-clock pacing
sim_end = 20.0                                             # total simulated time (s)
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                      # start the frame
    vis.Render()                                          # draw the scene
    vis.EndScene()                                        # finish the frame
    for _ in range(render_every):
        time = system.GetChTime()                         # current sim time
        driver_inputs = driver.GetInputs()               # poll the driver inputs

        driver.Synchronize(time)                          # update driver
        terrain.Synchronize(time)                         # update terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)  # feed inputs + terrain to the vehicle
        vis.Synchronize(time, driver_inputs)             # update visualization

        driver.Advance(step_size)                         # advance driver
        terrain.Advance(step_size)                        # advance terrain
        hmmwv.Advance(step_size)                          # advance the wrapper-owned system
        vis.Advance(step_size)                            # advance visualization

        realtime_timer.Spin(step_size)                    # pace to wall clock
        if system.GetChTime() >= sim_end:
            break
