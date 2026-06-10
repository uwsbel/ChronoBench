import os                                                              # review-only env flag
import math                                                           # render-cadence math
import pychrono.core as chrono                                        # core PyChrono API
import pychrono.vehicle as veh                                        # wheeled-vehicle catalog
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(-10, -2, 0.6)                           # HMMWV start position (on flat patch)
init_rot = chrono.QuatFromAngleZ(0)                                  # heading along +X
step_size = 2e-3                                                     # integration step (s)
tire_step_size = 1e-3                                                # tire force step (s)

hmmwv = veh.HMMWV_Full()                                             # full HMMWV model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
hmmwv.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # spawn pose
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                     # prompt: engine type (shafts engine)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # paired automatic shafts transmission
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                        # prompt: drivetrain type (all-wheel drive)
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                              # tire integration step
hmmwv.Initialize()                                                  # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)    # mesh visuals on chassis
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH) # mesh visuals on suspension
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)   # mesh visuals on steering
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)      # mesh visuals on wheels
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)       # mesh visuals on tires

system = hmmwv.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                 # multi-patch rigid terrain

patch1_mat = chrono.ChContactMaterialNSC()                         # material for flat patch 1
patch1_mat.SetFriction(0.9)                                        # high grip
patch1_mat.SetRestitution(0.01)                                    # nearly inelastic
patch1 = terrain.AddPatch(patch1_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(-16, 0, 0), chrono.QUNIT),
                          32, 20)                                  # flat patch 32 x 20 m
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # sandy tint
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)  # tiled tile texture

patch2_mat = chrono.ChContactMaterialNSC()                         # material for flat patch 2
patch2_mat.SetFriction(0.9)                                        # high grip
patch2_mat.SetRestitution(0.01)                                    # nearly inelastic
patch2 = terrain.AddPatch(patch2_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(16, 0, 0), chrono.QUNIT),
                          32, 30)                                  # second flat patch, flush with the first
patch2.SetColor(chrono.ChColor(1.0, 1.0, 1.0))                     # bright concrete tint
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)  # different texture

patch3_mat = chrono.ChContactMaterialNSC()                         # material for the mesh bump patch
patch3_mat.SetFriction(0.9)                                        # high grip
patch3_mat.SetRestitution(0.01)                                    # nearly inelastic
patch3 = terrain.AddPatch(patch3_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/meshes/bump.obj"))  # mesh-based bump patch
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))                     # bluish tint
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture

patch4_mat = chrono.ChContactMaterialNSC()                         # material for the heightmap patch
patch4_mat.SetFriction(0.9)                                        # high grip
patch4_mat.SetRestitution(0.01)                                    # nearly inelastic
patch4 = terrain.AddPatch(patch4_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(0, 42, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/bump64.bmp"),
                          64, 64, 0.0, 3.0)                        # heightmap patch with varying elevation
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)  # grass texture

terrain.Initialize()                                               # build all terrain patches

render_step_size = 1.0 / 50.0                                      # 50 FPS render cadence
render_steps = math.ceil(render_step_size / step_size)            # physics steps between frames

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle-specific Irrlicht window
vis.SetWindowTitle("HMMWV on Multi-Patch Rigid Terrain")         # window title
vis.SetWindowSize(1280, 1024)                                     # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)      # chase camera behind the chassis
vis.Initialize()                                                  # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo overlay
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                         # directional light (vehicle truths use this)
vis.AttachVehicle(hmmwv.GetVehicle())                            # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive steering/throttle/braking
steering_time = 1.0                                               # s to reach full steering
throttle_time = 1.0                                               # s to reach full throttle
braking_time = 0.3                                                # s to reach full braking
driver.SetSteeringDelta(render_step_size / steering_time)        # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)        # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)          # braking ramp rate
driver.Initialize()                                              # arm the driver

render_every = max(1, render_steps)                              # untagged cadence constant
sim_end = 12.0                                                   # review-only run horizon

realtime_timer = chrono.ChRealtimeStepTimer()                   # wall-clock pacing
step_number = 0                                                 # physics step counter
while vis.Run():
    time = hmmwv.GetSystem().GetChTime()                       # current sim time

    if step_number % render_steps == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                         # current steering/throttle/braking


    driver.Synchronize(time)                                   # advance driver state
    terrain.Synchronize(time)                                  # advance terrain state
    hmmwv.Synchronize(time, driver_inputs, terrain)            # feed inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                       # update HUD / camera

    driver.Advance(step_size)                                  # integrate driver
    terrain.Advance(step_size)                                 # integrate terrain
    hmmwv.Advance(step_size)                                   # integrate wrapper-owned system
    vis.Advance(step_size)                                     # integrate visualization


    step_number += 1                                           # advance step counter
    realtime_timer.Spin(step_size)                            # spin so wall-clock matches sim time
