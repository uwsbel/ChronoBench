import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 5e-4                                                       # small step for stable tracked contact at low spawn
init_loc = chrono.ChVector3d(-5, 0, 0.5)                              # vehicle spawn location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # no initial rotation
vis_type = veh.VisualizationType_PRIMITIVES                          # subsystem visualization style

vehicle = veh.M113()                                                  # M113 tracked vehicle wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)                 # M113 truth uses SMC contact
vehicle.SetChassisFixed(False)                                       # MANDATORY — fixed chassis won't move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)              # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)                   # tracked driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)                   # shafts engine model
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)   # automatic shafts transmission
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)                          # simple brake model
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))    # place the vehicle in the world
vehicle.Initialize()                                                 # build the tracked vehicle

vehicle.SetChassisVisualizationType(vis_type)                       # chassis visuals
vehicle.SetSprocketVisualizationType(vis_type)                     # sprocket visuals
vehicle.SetIdlerVisualizationType(vis_type)                        # idler visuals
vehicle.SetSuspensionVisualizationType(vis_type)                   # suspension visuals
vehicle.SetRoadWheelVisualizationType(vis_type)                    # road wheel visuals
vehicle.SetTrackShoeVisualizationType(vis_type)                    # track shoe visuals

system = vehicle.GetSystem()                                         # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)          # stable solver for tracked contact

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # flat rigid ground under the vehicle
patch_mat = chrono.ChContactMaterialSMC()                           # SMC material to match the system
patch_mat.SetFriction(0.9)                                          # ground friction
patch_mat.SetRestitution(0.01)                                      # ground restitution
patch_mat.SetYoungModulus(2e7)                                      # contact stiffness
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100x100 m flat patch at origin
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # tan ground color
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled ground texture
terrain.Initialize()                                                 # build the terrain

box_mat = chrono.ChContactMaterialSMC()                              # contact material for the obstacle box
box_mat.SetFriction(0.9)                                            # box friction
box_mat.SetRestitution(0.01)                                        # box restitution
box_mat.SetYoungModulus(2e7)                                        # box contact stiffness
box = chrono.ChBodyEasyBox(10.0, 1.0, 0.2, 1000, True, True, box_mat)  # long box to test vehicle mobility
box.SetPos(chrono.ChVector3d(5, 0, 0.1))                            # ahead of the vehicle, resting on ground
box.SetFixed(True)                                                  # anchored obstacle
box.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.2, 0.2))      # red box for visibility
system.AddBody(box)                                                 # add the box to the scene

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()                    # tracked-vehicle Irrlicht visual system
vis.SetWindowTitle("M113 Tracked Vehicle")                          # window title
vis.SetWindowSize(1280, 1024)                                       # window pixel size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0), 6.0, 0.5)          # chase camera tracking the chassis
vis.Initialize()                                                    # build the device (call first)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # corner logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # directional light (vehicle truth style)
vis.AttachVehicle(vehicle.GetVehicle())                            # bind the vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver bound to the vis
driver.SetSteeringDelta(1.0 / 50.0 / 1.0)                          # steering ramp rate
driver.SetThrottleDelta(1.0 / 50.0 / 1.0)                          # throttle ramp rate
driver.SetBrakingDelta(1.0 / 50.0 / 0.3)                           # braking ramp rate
driver.Initialize()                                                 # build the driver

render_step_size = 1.0 / 50.0                                       # render one frame every 1/50 s
render_steps = math.ceil(render_step_size / step_size)             # physics steps between frames


realtime_timer = chrono.ChRealtimeStepTimer()                       # spin to match wall-clock to sim time
step_number = 0                                                     # physics step counter
while vis.Run():
    time = system.GetChTime()                                       # current simulation time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver command struct
    driver_inputs.m_throttle = 0.8                                 # hard-coded throttle to test mobility
    driver_inputs.m_steering = 0.0                                 # drive straight
    driver_inputs.m_braking = 0.0                                  # no braking

    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    vehicle.Synchronize(time, driver_inputs)                       # tracked: 2-arg synchronize
    vis.Synchronize(time, driver_inputs)                           # update visualization


    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    vehicle.Advance(step_size)                                     # advances the wrapper-owned system
    vis.Advance(step_size)                                         # advance visualization

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # spin in place to real time
