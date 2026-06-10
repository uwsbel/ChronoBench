import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.8)                              # chassis spawn above terrain
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation
init_yaw = chrono.QuatFromAngleZ(0.0)                                # no initial heading

step_size = 1e-3                                                     # integration time step (s)
vis_type = veh.VisualizationType_MESH                              # mesh visuals for vehicle parts

vehicle = veh.M113()                                                # M113 tracked vehicle wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)               # M113 truth uses SMC
vehicle.SetChassisFixed(False)                                     # MANDATORY — fixed chassis won't move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)             # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)                  # tracked-vehicle driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)                  # shafts-based engine model
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shafts transmission
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)                         # simple brake model
vehicle.SetInitFwdVel(0.0)                                        # start from rest
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))   # spawn pose
vehicle.Initialize()                                              # build the tracked vehicle

vehicle.SetChassisVisualizationType(vis_type)                     # chassis mesh
vehicle.SetSprocketVisualizationType(vis_type)                    # sprocket mesh
vehicle.SetIdlerVisualizationType(vis_type)                       # idler mesh
vehicle.SetIdlerWheelVisualizationType(vis_type)                  # idler-wheel mesh
vehicle.SetSuspensionVisualizationType(vis_type)                  # suspension mesh
vehicle.SetRoadWheelVisualizationType(vis_type)                   # road-wheel mesh
vehicle.SetTrackShoeVisualizationType(vis_type)                   # track-shoe mesh

system = vehicle.GetSystem()                                      # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)       # stable solver for tracked contact
system.GetSolver().AsIterative().SetMaxIterations(150)           # more iterations keep track contact stable
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())          # report total vehicle mass

terrain = veh.RigidTerrain(system)                               # rigid ground under the vehicle
patch_mat = chrono.ChContactMaterialSMC()                        # SMC material (matches contact method)
patch_mat.SetFriction(0.9)                                       # tire/track-ground friction
patch_mat.SetRestitution(0.01)                                   # near-zero bounciness
patch_mat.SetYoungModulus(2e7)                                   # SMC contact stiffness
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200x200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                    # sandy ground color
terrain.Initialize()                                            # build terrain

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()                # tracked-vehicle Irrlicht visual
vis.SetWindowTitle("M113 Tracked Vehicle")                      # window title
vis.SetWindowSize(1280, 1024)                                   # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)    # chase-camera track point/distance/height
vis.Initialize()                                              # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # branding logo
vis.AddSkyBox()                                              # sky box backdrop
vis.AddLightDirectional()                                   # directional light (vehicle truth default)
vis.AttachVehicle(vehicle.GetVehicle())                     # bind chassis/track visual assets

driver_data = veh.vector_Entry([                           # scripted (time, steering, throttle, brake)
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),               # settle on the terrain first
    veh.DataDriverEntry(0.5, 0.0, 0.0, 0.0),               # hold before driving
    veh.DataDriverEntry(1.0, 0.0, 0.3, 0.0),               # ramp to a steady forward throttle
    veh.DataDriverEntry(10.0, 0.0, 0.3, 0.0),              # cruise forward to the end
])
driver = veh.ChDataDriver(vehicle.GetVehicle(), driver_data)  # scripted driver controlling the vehicle
driver.Initialize()                                      # finalize driver

time_step = step_size                                    # loop step
sim_end = 10.0                                          # simulation horizon (s)
render_fps = 50.0                                       # frames per second for capture
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()          # wall-clock pacing

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()                       # current sim time
        driver_inputs = driver.GetInputs()              # latest driver command

        driver.Synchronize(time)                        # update driver
        terrain.Synchronize(time)                       # update terrain
        vehicle.Synchronize(time, driver_inputs)        # tracked: 2-arg synchronize (no terrain)
        vis.Synchronize(time, driver_inputs)            # update visual HUD

        driver.Advance(time_step)                       # advance driver
        terrain.Advance(time_step)                      # advance terrain
        vehicle.Advance(time_step)                      # advances the wrapper-owned system
        vis.Advance(time_step)                          # advance visual

        realtime_timer.Spin(time_step)                  # spin so wall-clock matches sim time
        if system.GetChTime() >= sim_end:
            break
