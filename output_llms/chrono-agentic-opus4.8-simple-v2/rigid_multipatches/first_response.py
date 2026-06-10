import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(-10, 0, 1.0)                            # vehicle spawn (front of patch layout)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation, facing +X
step_size = 2e-3                                                     # integration step
tire_step_size = 1e-3                                                # tire force model substep

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision shape
hmmwv.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial pose
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                    # prompt: engine type
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # matching transmission
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                        # prompt: drivetrain type (all-wheel drive)
hmmwv.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tire model on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                             # tire substep
hmmwv.Initialize()                                                  # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)   # mesh on all components
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                          # take wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                 # rigid multi-patch terrain

patch1_mat = chrono.ChContactMaterialNSC()                         # material for the flat tiled patch
patch1_mat.SetFriction(0.9)                                        # high grip
patch1_mat.SetRestitution(0.01)                                    # little bounce
patch1 = terrain.AddPatch(patch1_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(-16, 0, 0), chrono.QUNIT),
                          32, 20)                                   # flat patch, 32 x 20 m
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)  # tile texture
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                     # light grey

patch2_mat = chrono.ChContactMaterialNSC()                         # second flat patch material
patch2_mat.SetFriction(0.9)
patch2_mat.SetRestitution(0.01)
patch2 = terrain.AddPatch(patch2_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(16, 0, 0.15), chrono.QUNIT),
                          32, 30)                                   # second flat patch, raised slightly
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)  # concrete texture
patch2.SetColor(chrono.ChColor(1.0, 1.0, 1.0))

patch3_mat = chrono.ChContactMaterialNSC()                         # mesh-based bump patch material
patch3_mat.SetFriction(0.9)
patch3_mat.SetRestitution(0.01)
patch3 = terrain.AddPatch(patch3_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/meshes/bump.obj"))  # mesh patch (bump)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6, 6)  # dirt texture on the bump

patch4_mat = chrono.ChContactMaterialNSC()                         # heightmap patch material
patch4_mat.SetFriction(0.9)
patch4_mat.SetRestitution(0.01)
patch4 = terrain.AddPatch(patch4_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(0, 42, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/test64.bmp"),
                          64, 64, 0, 3)                             # heightmap patch, elevation 0..3 m
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)  # grass texture

terrain.Initialize()                                               # finalize all patches

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-aware Irrlicht visual system
vis.SetWindowTitle("HMMWV on Multi-Patch Rigid Terrain")          # window title
vis.SetWindowSize(1280, 1024)                                      # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)       # chase camera tracking the chassis
vis.Initialize()                                                  # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                         # vehicle demos use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                            # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive keyboard driver
steering_time = 1.0                                               # seconds 0 -> +1 steering
throttle_time = 1.0                                               # seconds 0 -> +1 throttle
braking_time = 0.3                                                # seconds 0 -> +1 brake
driver.SetSteeringDelta(step_size / steering_time)               # steering ramp rate
driver.SetThrottleDelta(step_size / throttle_time)               # throttle ramp rate
driver.SetBrakingDelta(step_size / braking_time)                 # braking ramp rate
driver.Initialize()                                              # start the driver

render_step_size = 1.0 / 50.0                                     # render at 50 FPS
render_steps = math.ceil(render_step_size / step_size)           # physics steps per rendered frame
render_every = render_steps                                      # untagged cadence constant
sim_end = 30.0                                                   # simulation duration (s)


realtime_timer = chrono.ChRealtimeStepTimer()                   # spin to match wall-clock
step_number = 0                                                  # loop step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                   # current sim time

    if step_number % render_steps == 0:                         # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                          # current driver commands

    driver.Synchronize(time)                                    # advance driver state
    terrain.Synchronize(time)                                   # advance terrain state
    hmmwv.Synchronize(time, driver_inputs, terrain)            # feed inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                       # sync HUD / chase camera

    driver.Advance(step_size)                                   # step driver
    terrain.Advance(step_size)                                  # step terrain
    hmmwv.Advance(step_size)                                    # step wrapper-owned system
    vis.Advance(step_size)                                      # step visualization


    step_number += 1                                            # advance step count
    realtime_timer.Spin(step_size)                             # spin in place to match real time
