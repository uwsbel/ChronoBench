import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

initLoc = chrono.ChVector3d(0, 0, 0.5)                              # initial chassis location
initRot = chrono.QuatFromAngleZ(0)                                   # no initial yaw

step_size = 1e-3                                                     # simulation time step (s)
render_fps = 50                                                       # target render rate (fps)
render_step_size = 1.0 / render_fps                                  # seconds per frame
render_steps = math.ceil(render_step_size / step_size)              # physics steps per render

terrainLength = 100.0                                                # terrain X dimension (m)
terrainWidth = 100.0                                                 # terrain Y dimension (m)

artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
artcar.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision mesh
artcar.SetChassisFixed(False)                                        # must move freely
artcar.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))        # spawn location
artcar.SetTireType(veh.TireModelType_RIGID)                          # rigid tires on rigid terrain
artcar.SetTireStepSize(step_size)                                    # tire integrator step
artcar.Initialize()                                                  # finalize vehicle

system = artcar.GetSystem()                                          # get the owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # enable Bullet collision

artcar.SetChassisVisualizationType(chrono.VisualizationType_MESH)    # mesh chassis visuals
artcar.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
artcar.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
artcar.SetWheelVisualizationType(chrono.VisualizationType_MESH)      # mesh wheels
artcar.SetTireVisualizationType(chrono.VisualizationType_MESH)       # mesh tires

print("VEHICLE MASS: ", artcar.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()                            # NSC material pairs with NSC vehicle
patch_mat.SetFriction(0.9)                                           # rolling friction
patch_mat.SetRestitution(0.01)                                       # low bounce
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,                                                 # flat at world origin
    terrainLength,
    terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()                                                 # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar on Rigid Terrain")                        # window label
vis.SetWindowSize(1280, 720)                                         # display resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 6.0, 0.5)          # chase camera offset
vis.Initialize()                                                     # create Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # Project Chrono logo
vis.AddSkyBox()                                                      # sky background
vis.AddLightDirectional()                                            # directional sunlight
vis.AttachVehicle(artcar.GetVehicle())                               # bind vehicle visuals

driver = veh.ChInteractiveDriverIRR(vis)                             # pass vis system, not vehicle
steering_time = 1.0                                                  # seconds to full steering deflection
throttle_time = 1.0                                                  # seconds to full throttle
braking_time = 0.3                                                   # seconds to full brake
driver.SetSteeringDelta(render_step_size / steering_time)            # steering rate per render step
driver.SetThrottleDelta(render_step_size / throttle_time)            # throttle rate per render step
driver.SetBrakingDelta(render_step_size / braking_time)              # braking rate per render step
driver.Initialize()                                                  # finalize driver

render_every = max(1, round(1.0 / (render_fps * step_size)))         # physics steps per rendered frame

realtime_timer = chrono.ChRealtimeStepTimer()                        # real-time pacing
step_number = 0
while vis.Run():
    time = system.GetChTime()                                        # current simulation time

    if step_number % render_steps == 0:                              # render at 50 fps cadence
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                               # read keyboard inputs


    driver.Synchronize(time)                                         # update driver state
    terrain.Synchronize(time)                                        # update terrain
    artcar.Synchronize(time, driver_inputs, terrain)                 # synchronize vehicle subsystems
    vis.Synchronize(time, driver_inputs)                             # update HUD/vis

    driver.Advance(step_size)                                        # advance driver
    terrain.Advance(step_size)                                       # advance terrain
    artcar.Advance(step_size)                                        # advance vehicle + owned system
    vis.Advance(step_size)                                           # advance vis

    step_number += 1
    realtime_timer.Spin(step_size)                                   # pace to real time
