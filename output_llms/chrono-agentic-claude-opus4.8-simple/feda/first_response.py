import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                             # spawn the FEDA on the patch
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # identity orientation (QUNIT)

step_size = 1e-3                                                     # dynamics step
tire_step_size = step_size                                          # tire force model step
render_step_size = 1.0 / 50.0                                        # 50 FPS render cadence

terrainLength = 100.0                                               # X size of rigid patch
terrainWidth = 100.0                                                # Y size of rigid patch

feda = veh.FEDA()                                                   # FED-Alpha catalog wrapper (owns its system)
feda.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
feda.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
feda.SetChassisFixed(False)                                        # MANDATORY — chassis must move
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # initial pose
feda.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY tire on rigid terrain
feda.SetTireStepSize(tire_step_size)                               # tire integration step
feda.Initialize()                                                  # build the vehicle

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)       # mesh visualization for all parts
feda.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
feda.SetSteeringVisualizationType(veh.VisualizationType_MESH)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

system = feda.GetSystem()                                          # the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                 # rigid terrain on the shared system
patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                         # friction coefficient
patch_mat.SetRestitution(0.01)                                     # near-inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # custom tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # patch tint
terrain.Initialize()                                              # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle-specific Irrlicht vis
vis.SetWindowTitle('FEDA vehicle')                               # window title
vis.SetWindowSize(1280, 1024)                                    # window size
vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)      # follow camera from behind
vis.Initialize()                                                 # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # logo
vis.AddLightDirectional()                                        # directional light
vis.AddSkyBox()                                                  # sky
vis.AttachVehicle(feda.GetVehicle())                            # bind the vehicle to the vis

driver = veh.ChInteractiveDriverIRR(vis)                         # interactive keyboard driver
steering_time = 1.0                                             # s to reach full steer
throttle_time = 1.0                                            # s to reach full throttle
braking_time = 0.3                                             # s to reach full brake
driver.SetSteeringDelta(render_step_size / steering_time)      # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)      # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)        # braking rate
driver.Initialize()                                            # build the driver

render_steps = math.ceil(render_step_size / step_size)         # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                  # wall-clock pacing
step_number = 0                                                # step counter


while vis.Run():                                               # real-time render loop
    time = feda.GetSystem().GetChTime()                        # current sim time

    if step_number % render_steps == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                         # current driver command

    driver.Synchronize(time)                                   # sync driver
    terrain.Synchronize(time)                                  # sync terrain
    feda.Synchronize(time, driver_inputs, terrain)             # sync vehicle
    vis.Synchronize(time, driver_inputs)                       # sync vis

    driver.Advance(step_size)                                  # advance driver
    terrain.Advance(step_size)                                 # advance terrain
    feda.Advance(step_size)                                    # advance wrapper-owned system
    vis.Advance(step_size)                                     # advance vis

    step_number += 1                                           # next step
    realtime_timer.Spin(step_size)                             # pace to wall-clock
