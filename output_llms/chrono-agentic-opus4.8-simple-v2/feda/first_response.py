import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate bundled vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                               # vehicle spawn location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # spawn orientation (identity, facing +X)
step_size = 1e-3                                                      # integration time step
tire_step_size = 1e-3                                                 # tire force model step

vis_type = veh.VisualizationType_MESH                               # mesh visualization for all vehicle parts

feda = veh.FEDA()                                                     # FED-Alpha catalog vehicle
feda.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
feda.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision shell
feda.SetChassisFixed(False)                                          # chassis must be free to move
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # set spawn pose
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)                  # FEDA truth powertrain
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)                           # FEDA Pacejka tire model
feda.SetTireStepSize(tire_step_size)                               # tire integration step
feda.Initialize()                                                    # build the vehicle subsystems

feda.SetChassisVisualizationType(vis_type)                          # mesh chassis
feda.SetSuspensionVisualizationType(vis_type)                       # mesh suspension
feda.SetSteeringVisualizationType(vis_type)                         # mesh steering
feda.SetWheelVisualizationType(vis_type)                            # mesh wheels
feda.SetTireVisualizationType(vis_type)                             # mesh tires

system = feda.GetSystem()                                            # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # collision system for terrain contact

print("VEHICLE MASS: ", feda.GetVehicle().GetMass())                # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # flat rigid terrain
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                          # tire-ground friction
patch_mat.SetRestitution(0.01)                                      # near-inelastic contact
terrainLength = 200.0                                               # terrain X size (m)
terrainWidth = 200.0                                                # terrain Y size (m)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)          # custom road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                      # neutral patch tint
terrain.Initialize()                                                # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle-specific Irrlicht window
vis.SetWindowTitle("FEDA on Rigid Terrain")                        # window title
vis.SetWindowSize(1280, 1024)                                      # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)   # follow camera: track point, distance, height
vis.Initialize()                                                    # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                           # directional light (vehicle truth style)
vis.AttachVehicle(feda.GetVehicle())                               # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                           # interactive keyboard driver
steering_time = 1.0                                                 # seconds 0 -> +1 steering
throttle_time = 1.0                                                 # seconds 0 -> +1 throttle
braking_time = 0.3                                                  # seconds 0 -> +1 braking
render_step_size = 1.0 / 50.0                                       # 50 frames per second
driver.SetSteeringDelta(render_step_size / steering_time)          # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)          # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)            # braking ramp rate
driver.Initialize()                                                 # finalize the driver

render_steps = math.ceil(render_step_size / step_size)             # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacer
step_number = 0                                                     # physics step counter
sim_end = 20.0                                                      # simulation duration (s)

while vis.Run() and feda.GetSystem().GetChTime() < sim_end:
    time = feda.GetSystem().GetChTime()                            # current sim time

    if step_number % render_steps == 0:                            # throttled rendering at 50 fps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                            # current steering/throttle/brake

    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    feda.Synchronize(time, driver_inputs, terrain)                # feed inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                          # update HUD/visuals

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    feda.Advance(step_size)                                        # advance wrapper-owned system
    vis.Advance(step_size)                                         # advance visuals


    step_number += 1                                               # advance step counter
    realtime_timer.Spin(step_size)                                # match wall-clock to sim time
