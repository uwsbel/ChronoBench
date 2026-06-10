import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                    # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # locate vehicle data files

step_size = 1e-3                                                        # integration step (s)
init_loc = chrono.ChVector3d(-5, 0, 0.5)                                # initial vehicle location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # no initial rotation

vehicle = veh.M113()                                                    # M113 tracked vehicle
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)                   # M113 truth uses SMC
vehicle.SetChassisFixed(False)                                         # MANDATORY - fixed chassis won't move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)                 # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)                      # tracked driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)                      # shafts engine model
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shafts transmission
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)                             # simple brake model
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # spawn pose
vehicle.Initialize()                                                    # build the vehicle

vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)   # chassis visuals
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)  # track-shoe visuals
vehicle.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)   # sprocket visuals
vehicle.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)      # idler visuals
vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)  # road-wheel visuals

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # required for contact
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)            # stable tracked-contact solver
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())               # report total vehicle mass

system = vehicle.GetSystem()                                            # wrapper-owned system

terrain = veh.RigidTerrain(system)                                      # rigid ground under the tracks
patch_mat = chrono.ChContactMaterialSMC()                              # SMC material to match the vehicle
patch_mat.SetFriction(0.9)                                             # terrain friction
patch_mat.SetRestitution(0.01)                                         # terrain restitution
patch_mat.SetYoungModulus(2e7)                                        # terrain stiffness
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)    # flat 100x100 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)   # tiled ground texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                         # ground color
terrain.Initialize()                                                    # build terrain

box_mat = chrono.ChContactMaterialSMC()                                # box contact material (SMC)
box_mat.SetFriction(0.9)                                               # box friction
box_mat.SetRestitution(0.01)                                           # box restitution
box_mat.SetYoungModulus(2e7)                                          # box stiffness
box = chrono.ChBodyEasyBox(20.0, 1.0, 0.2, 1000, True, True, box_mat)   # long low box to test mobility
box.SetPos(chrono.ChVector3d(10, 0, 0.1))                             # low obstacle ahead, clear of the spawn
box.SetFixed(True)                                                     # anchored obstacle
system.AddBody(box)                                                    # add to the shared system

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()                       # tracked-vehicle Irrlicht system
vis.SetWindowTitle("M113 Tracked Vehicle")                            # window title
vis.SetWindowSize(1280, 1024)                                         # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0), 8.0, 1.5)             # follow the chassis
vis.Initialize()                                                       # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))      # logo
vis.AddSkyBox()                                                        # sky box
vis.AddLightDirectional()                                             # vehicle demos use a directional light
vis.AttachVehicle(vehicle.GetVehicle())                               # bind chassis/track visuals

driver = veh.ChInteractiveDriverIRR(vis)                               # interactive driver bound to the vis
driver.SetSteeringDelta(0.02)                                         # steering ramp rate
driver.SetThrottleDelta(0.02)                                         # throttle ramp rate
driver.SetBrakingDelta(0.06)                                          # braking ramp rate
driver.Initialize()                                                    # build the driver

sim_end = 10.0                                                         # simulation duration (s)
render_step_size = 1.0 / 50.0                                          # render cadence (s)
render_steps = math.ceil(render_step_size / step_size)                # physics steps per frame
render_every = max(1, render_steps)                                   # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                         # spin to wall-clock
step_number = 0                                                       # step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                        # current sim time

    if step_number % render_steps == 0:                             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver.SetThrottle(0.8)                                          # hard-coded throttle 0.8
    driver_inputs = driver.GetInputs()                              # current driver inputs

    driver.Synchronize(time)                                        # update driver
    terrain.Synchronize(time)                                       # update terrain
    vehicle.Synchronize(time, driver_inputs)                        # tracked: 2-arg synchronize
    vis.Synchronize(time, driver_inputs)                            # update visuals


    driver.Advance(step_size)                                       # advance driver
    terrain.Advance(step_size)                                      # advance terrain
    vehicle.Advance(step_size)                                      # advances the wrapper-owned system
    vis.Advance(step_size)                                          # advance visuals

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # match wall-clock to sim time
