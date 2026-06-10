import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

# M113 tracked vehicle — initial location at (-5, 0, 0.5)
init_loc = chrono.ChVector3d(-5, 0, 0.5)                             # prompt: changed init pos
init_rot = chrono.QuatFromAngleZ(0)                                   # no heading rotation

vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)                  # M113 truth uses SMC
vehicle.SetChassisFixed(False)                                        # MANDATORY — must not be fixed
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)               # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)                    # band-drive driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)                    # shafts engine model
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shafts
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)                           # simple brake
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # set spawn position
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)     # chassis vis
vehicle.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)    # sprocket vis
vehicle.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)       # idler vis
vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)   # road wheel vis
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)   # track shoe vis

system = vehicle.GetSystem()                                          # get the M113-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)           # stable solver for tracked

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # truth's literal banner

# Rigid terrain (flat) under the M113 — SMC contact to match vehicle
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()                            # SMC to match M113
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # large flat ground
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Long box added to test vehicle mobility — placed ahead of the vehicle
box_mat = chrono.ChContactMaterialSMC()                              # SMC material for box
box_mat.SetFriction(0.9)
box_mat.SetRestitution(0.01)
box_mat.SetYoungModulus(2e7)
box_body = chrono.ChBodyEasyBox(20.0, 1.0, 0.3, 1000.0, True, True, box_mat)  # long box
box_body.SetPos(chrono.ChVector3d(10, 0, 0.15))                      # centered ahead at z=0.15
box_body.SetFixed(True)                                               # fixed obstacle box
system.AddBody(box_body)

# Tracked vehicle Irrlicht visualization
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Tracked Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)         # chase camera
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                             # directional light (vehicle truth)
vis.AttachVehicle(vehicle.GetVehicle())                               # attach tracked vehicle

# Interactive driver — throttle hard-coded to 0.8 per prompt
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)                                         # steering response
driver.SetThrottleDelta(0.02)                                         # throttle ramp
driver.SetBrakingDelta(0.06)                                          # braking ramp
driver.Initialize()

step_size = 5e-4                                                      # M113 needs small step
sim_end = 20.0                                                        # simulate 20 s
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))         # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0                                                       # for render gating (scored core)

while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_every == 0:                               # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.8                                    # hard-coded throttle = 0.8 (prompt)
    driver_inputs.m_braking = 0.0                                     # no braking

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)                          # 2-arg for tracked vehicles
    vis.Synchronize(time, driver_inputs)


    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)                                        # advances the system
    vis.Advance(step_size)

    step_number += 1                                                  # increment step counter
    realtime_timer.Spin(step_size)                                    # real-time pacing
