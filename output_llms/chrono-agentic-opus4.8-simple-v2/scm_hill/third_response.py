import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                     # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')                 # locate vehicle data files

init_loc = chrono.ChVector3d(-5, 0, 0.6)                                 # spawn over the hill heightmap
init_rot = chrono.QuatFromAngleZ(0)                                      # facing +X
step_size = 2e-3                                                         # vehicle dynamics step
tire_step_size = 1e-3                                                    # tire substep
sim_end = 20.0                                                           # total simulated seconds

hmmwv = veh.HMMWV_Full()                                                 # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                       # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                    # no chassis collision mesh
hmmwv.SetChassisFixed(False)                                            # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))           # initial pose
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                          # shaft-based engine
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)   # automatic shaft transmission
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                              # all-wheel drive
hmmwv.SetTireType(veh.TireModelType_TMEASY)                             # TMeasy tires on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                                   # tire integration step
hmmwv.Initialize()                                                      # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)          # mesh chassis
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # primitive suspension
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # primitive steering
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)            # mesh wheels
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)             # mesh tires

system = hmmwv.GetSystem()                                              # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # REQUIRED, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                   # report total vehicle mass

terrain = veh.RigidTerrain(system)                                      # rigid terrain on the shared system

patch_mat = chrono.ChContactMaterialNSC()                               # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                              # tire-ground friction
patch_mat.SetRestitution(0.01)                                          # nearly inelastic ground

patch = terrain.AddPatch(                                               # single height-map patch
    patch_mat,
    chrono.CSYSNORM,                                                    # centered at origin, no rotation
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),                  # bump hill height map
    64, 64,                                                             # patch length, width (m)
    0.0, 1.0,                                                           # hMin, hMax (m) elevation range
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 64, 64)  # dirt texture, UV tiling
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                           # earthy tint

terrain.Initialize()                                                    # build the rigid terrain body

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                        # vehicle-aware Irrlicht window
vis.SetWindowTitle("HMMWV on Rigid Hill Terrain")                       # window title
vis.SetWindowSize(1280, 1024)                                          # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)           # chase cam: track point, dist, height
vis.Initialize()                                                       # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))       # logo overlay
vis.AddSkyBox()                                                        # sky box
vis.AddLightDirectional()                                             # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                                 # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                               # interactive (keyboard) driver
steering_time = 1.0                                                    # s to ramp steering 0 -> 1
throttle_time = 1.0                                                    # s to ramp throttle 0 -> 1
braking_time = 0.3                                                     # s to ramp brake 0 -> 1
render_step_size = 1.0 / 50.0                                          # 50 fps render cadence
driver.SetSteeringDelta(render_step_size / steering_time)             # steering rate limit
driver.SetThrottleDelta(render_step_size / throttle_time)             # throttle rate limit
driver.SetBrakingDelta(render_step_size / braking_time)               # brake rate limit
driver.Initialize()                                                   # build the driver

render_steps = math.ceil(render_step_size / step_size)                # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                         # wall-clock pacing
step_number = 0                                                       # physics step counter


while vis.Run():
    time = hmmwv.GetSystem().GetChTime()                             # current sim time

    if step_number % render_steps == 0:                             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                              # poll driver inputs

    driver.Synchronize(time)                                        # advance driver clock
    terrain.Synchronize(time)                                       # advance terrain clock
    hmmwv.Synchronize(time, driver_inputs, terrain)                # feed inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                           # update HUD / camera

    driver.Advance(step_size)                                       # step driver
    terrain.Advance(step_size)                                      # step terrain
    hmmwv.Advance(step_size)                                        # advances the wrapper-owned system
    vis.Advance(step_size)                                          # step visualization


    step_number += 1                                               # advance counter
    realtime_timer.Spin(step_size)                                 # spin so wall-clock matches sim time

    if time >= sim_end:                                            # stop at the end time
        break
