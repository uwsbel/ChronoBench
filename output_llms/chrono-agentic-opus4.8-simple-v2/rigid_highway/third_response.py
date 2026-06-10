import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate bundled vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn (geometric-center origin)
init_rot = chrono.QuatFromAngleZ(0)                                 # facing +X
step_size = 2e-3                                                     # integration step
tire_step_size = 1e-3                                               # tire substep

hmmwv = veh.HMMWV_Full()                                            # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)             # chassis collision off (terrain only)
hmmwv.SetChassisFixed(False)                                       # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))     # world spawn pose
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                  # shafts engine model
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shafts transmission
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                      # all-wheel drive
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)            # pitman-arm steering
hmmwv.SetBrakeType(veh.BrakeType_SHAFTS)                         # shafts brake model
hmmwv.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tire on rigid road
hmmwv.SetTireStepSize(tire_step_size)                            # tire force substep
hmmwv.SetAerodynamicDrag(0.5, 5.0, 1.2)                         # drag coeff, frontal area, air density
hmmwv.Initialize()                                                 # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)  # mesh chassis
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # primitive suspension
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # primitive steering
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)    # mesh wheels
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)     # mesh tires

system = hmmwv.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # rigid terrain on the vehicle system

patch_mat = chrono.ChContactMaterialNSC()                          # NSC patch contact material
patch_mat.SetFriction(0.4)                                          # friction coefficient (updated 0.9 -> 0.4)
patch_mat.SetRestitution(0.05)                                      # restitution (updated 0.01 -> 0.05)

terrainLength = 200.0                                               # patch size along its local X (m)
terrainWidth = 200.0                                                # patch size along its local Y (m)
patch_csys = chrono.ChCoordsysd(                                    # patch pose at the cross roads
    chrono.ChVector3d(6, -70, 0),                                  # patch position (6, -70, 0)
    chrono.QuatFromAngleZ(-90 * chrono.CH_DEG_TO_RAD),            # -90 deg rotation about Z
)
patch = terrain.AddPatch(patch_mat, patch_csys, terrainLength, terrainWidth)  # flat sized patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)     # tiled road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # patch color
terrain.Initialize()                                               # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-specific Irrlicht system
vis.SetWindowTitle("Rigid Highway")                                # window title
vis.SetWindowSize(1280, 1024)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)      # chase camera offsets
vis.Initialize()                                                   # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                                    # sky box
vis.AddLightDirectional()                                          # directional light (vehicle truths)
vis.AttachVehicle(hmmwv.GetVehicle())                             # bind vehicle visuals

driver = veh.ChInteractiveDriverIRR(vis)                           # interactive driver bound to vis
steering_time = 1.0                                                # s to full steering
throttle_time = 1.0                                                # s to full throttle
braking_time = 0.3                                                 # s to full brake
driver.SetSteeringDelta(step_size / steering_time)               # steering ramp rate
driver.SetThrottleDelta(step_size / throttle_time)               # throttle ramp rate
driver.SetBrakingDelta(step_size / braking_time)                 # braking ramp rate
driver.Initialize()                                               # finalize driver

render_step_size = 1.0 / 50.0                                      # 50 FPS render cadence
render_steps = math.ceil(render_step_size / step_size)           # physics steps per rendered frame
render_every = render_steps                                       # untagged cadence constant


realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
step_number = 0                                                   # step counter
while vis.Run():
    time = hmmwv.GetSystem().GetChTime()                          # current sim time

    if step_number % render_steps == 0:                          # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                           # current driver inputs

    driver.Synchronize(time)                                     # sync driver
    terrain.Synchronize(time)                                    # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)            # sync vehicle with terrain
    vis.Synchronize(time, driver_inputs)                       # sync visuals

    driver.Advance(step_size)                                    # advance driver
    terrain.Advance(step_size)                                   # advance terrain
    hmmwv.Advance(step_size)                                     # advances the wrapper-owned system
    vis.Advance(step_size)                                       # advance visuals


    step_number += 1                                             # advance step counter
    realtime_timer.Spin(step_size)                              # spin so wall-clock matches sim time
    if time >= 12.0:                                            # stop after 12 s
        break
