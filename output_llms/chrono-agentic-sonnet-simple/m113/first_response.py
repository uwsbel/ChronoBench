import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle data files

initLoc = chrono.ChVector3d(0, 0, 1.1)                              # initial chassis position
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                          # no initial rotation

vis_type = veh.VisualizationType_MESH                                # mesh visualization
chassis_collision_type = veh.CollisionType_NONE                      # no chassis collision

terrainHeight = 0                                                    # terrain height (m)
terrainLength = 100.0                                                # terrain X size (m)
terrainWidth = 100.0                                                 # terrain Y size (m)

trackPoint = chrono.ChVector3d(0.0, 0.0, 0.1)                       # camera chase point

contact_method = chrono.ChContactMethod_SMC                          # SMC contact for M113 truth
step_size = 5e-4                                                     # physics timestep (s)
render_step_size = 1.0 / 50                                          # 50 fps render interval
sim_end = 20.0                                                       # simulation end time (s)

vehicle = veh.M113()                                                 # create M113 tracked vehicle
vehicle.SetContactMethod(contact_method)                             # SMC contact method
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)              # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)                   # BDS driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)                   # shafts engine model
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shafts transmission
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)                          # simple brake model

vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))       # set initial pose
vehicle.Initialize()                                                 # build vehicle internals

vehicle.SetChassisVisualizationType(vis_type)                       # chassis visualization
vehicle.SetSprocketVisualizationType(vis_type)                      # sprocket visualization
vehicle.SetIdlerVisualizationType(vis_type)                         # idler visualization
vehicle.SetIdlerWheelVisualizationType(vis_type)                    # idler wheel visualization
vehicle.SetSuspensionVisualizationType(vis_type)                    # suspension visualization
vehicle.SetRoadWheelVisualizationType(vis_type)                     # road wheel visualization
vehicle.SetTrackShoeVisualizationType(vis_type)                     # track shoe visualization

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Bullet collision
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)           # solver for tracked contact

patch_mat = chrono.ChContactMaterialSMC()                           # SMC terrain material
patch_mat.SetFriction(0.9)                                          # friction coefficient
patch_mat.SetRestitution(0.01)                                      # low restitution (near rigid)
terrain = veh.RigidTerrain(vehicle.GetSystem())                     # rigid flat terrain
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),  # centered at origin
    terrainLength, terrainWidth)                                     # 100x100 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # terrain color
terrain.Initialize()                                                 # finalize terrain

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()                    # tracked-vehicle Irrlicht vis
vis.SetWindowTitle('M113 Demo')                                      # window title
vis.SetWindowSize(1280, 1024)                                        # window dimensions
vis.SetChaseCamera(trackPoint, 9.0, 1.5)                            # chase camera (track point, dist, height)
vis.Initialize()                                                     # create Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))    # Chrono logo (after Initialize)
vis.AddLightDirectional()                                            # directional light (vehicle style)
vis.AddSkyBox()                                                      # sky background
vis.AttachVehicle(vehicle.GetVehicle())                              # bind vehicle to vis

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive keyboard driver
steering_time = 1.0                                                  # s to go 0->+1 steering
throttle_time = 1.0                                                  # s to go 0->+1 throttle
braking_time = 0.3                                                   # s to go 0->+1 braking
driver.SetSteeringDelta(render_step_size / steering_time)           # steering increment per render step
driver.SetThrottleDelta(render_step_size / throttle_time)           # throttle increment per render step
driver.SetBrakingDelta(render_step_size / braking_time)             # braking increment per render step
driver.Initialize()                                                  # finalize driver

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # report vehicle mass

render_steps = math.ceil(render_step_size / step_size)              # physics steps per render frame

render_every = max(1, render_steps)                                  # untagged cadence constant

step_number = 0                                                      # step counter
vehicle.GetVehicle().EnableRealtime(True)                           # enable real-time pacing

while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    time = vehicle.GetSystem().GetChTime()                          # current sim time

    if step_number % render_steps == 0:                             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                              # get driver commands


    driver.Synchronize(time)                                        # sync driver
    terrain.Synchronize(time)                                       # sync terrain
    vehicle.Synchronize(time, driver_inputs)                        # sync vehicle (2-arg for tracked)
    vis.Synchronize(time, driver_inputs)                            # sync vis

    driver.Advance(step_size)                                       # advance driver
    terrain.Advance(step_size)                                      # advance terrain
    vehicle.Advance(step_size)                                      # advance vehicle (steps system)
    vis.Advance(step_size)                                          # advance vis

    step_number += 1                                                # increment step counter
