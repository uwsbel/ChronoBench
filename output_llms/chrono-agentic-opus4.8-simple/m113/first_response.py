import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.8)                              # initial chassis location
init_rot = chrono.QuatFromAngleZ(0)                                  # initial heading (facing +X)
step_size = 1e-3                                                     # integration step (s)
sim_end = 12.0                                                       # total simulated time (s)

vis_type = veh.VisualizationType_MESH                              # mesh visualization for the tracked vehicle

vehicle = veh.M113()                                                # M113 tracked vehicle
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)                # M113 truth uses SMC
vehicle.SetChassisFixed(False)                                      # MANDATORY — fixed chassis won't move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)             # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)                  # tracked-vehicle driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)                  # shaft-based engine
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic transmission
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)                         # simple brake model
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))   # initial pose
vehicle.Initialize()                                               # build the vehicle subsystems

vehicle.SetChassisVisualizationType(vis_type)                      # chassis mesh
vehicle.SetSprocketVisualizationType(vis_type)                     # sprocket mesh
vehicle.SetIdlerVisualizationType(vis_type)                        # idler mesh
vehicle.SetRoadWheelVisualizationType(vis_type)                    # road-wheel mesh
vehicle.SetTrackShoeVisualizationType(vis_type)                    # track-shoe mesh

system = vehicle.GetSystem()                                       # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)         # stable solver for tracked contact

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())           # report total vehicle mass

terrain = veh.RigidTerrain(system)                                # rigid ground under the vehicle
patch_mat = chrono.ChContactMaterialSMC()                         # SMC material (matches SMC system)
patch_mat.SetFriction(0.8)                                        # terrain friction
patch_mat.SetRestitution(0.01)                                    # terrain restitution
patch_mat.SetYoungModulus(2e7)                                    # contact stiffness
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # flat 200x200 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # ground texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # ground color
terrain.Initialize()                                              # finalize terrain

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()                  # tracked-vehicle Irrlicht window
vis.SetWindowTitle("M113 Tracked Vehicle")                        # window title
vis.SetWindowSize(1280, 1024)                                     # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)     # chase camera behind the vehicle
vis.Initialize()                                                  # create the Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                        # directional light (vehicle truths)
vis.AttachVehicle(vehicle.GetVehicle())                          # bind the vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive driver bound to the vis
steering_time = 1.0                                              # s to reach full steering
throttle_time = 1.0                                             # s to reach full throttle
braking_time = 0.3                                              # s to reach full brake
render_step_size = 1.0 / 50.0                                   # render cadence (s)
driver.SetSteeringDelta(render_step_size / steering_time)       # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)       # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)         # brake ramp rate
driver.Initialize()                                            # finalize driver

render_steps = math.ceil(render_step_size / step_size)          # physics steps per rendered frame
render_every = max(1, render_steps)                             # untagged cadence constant


realtime_timer = chrono.ChRealtimeStepTimer()                   # spin to keep wall-clock = sim time
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                  # current simulation time

    vis.BeginScene()                                           # render once per frame
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        time = system.GetChTime()                              # step-local time
        driver.SetThrottle(0.8)                                # m113 truth scripts 0.8 throttle
        driver_inputs = driver.GetInputs()                     # gather driver commands


        driver.Synchronize(time)                               # update driver
        terrain.Synchronize(time)                              # update terrain
        vehicle.Synchronize(time, driver_inputs)              # tracked vehicle: 2-arg Synchronize
        vis.Synchronize(time, driver_inputs)                  # update visualization

        driver.Advance(step_size)                              # advance driver
        terrain.Advance(step_size)                             # advance terrain
        vehicle.Advance(step_size)                             # advance vehicle (steps the system)
        vis.Advance(step_size)                                 # advance visualization

        realtime_timer.Spin(step_size)                         # spin in place for real-time pacing
        if system.GetChTime() >= sim_end:
            break
