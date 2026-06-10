import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # bundled data root
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data root

initLoc = chrono.ChVector3d(0, 0, 0.5)                               # spawn location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                          # QUNIT, no rotation

step_size = 1e-3                                                     # integration step
tire_step_size = step_size                                          # tire substep
render_step_size = 1.0 / 50.0                                        # 50 FPS real time

terrainLength = 100.0                                               # terrain X size
terrainWidth = 100.0                                                # terrain Y size

gator = veh.Gator()                                                 # catalog Gator wrapper
gator.SetContactMethod(chrono.ChContactMethod_NSC)                  # rigid-terrain NSC
gator.SetChassisCollisionType(veh.CollisionType_NONE)              # add primitive collision inline below
gator.SetChassisFixed(False)                                       # chassis free to move
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))        # initial pose
gator.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire
gator.SetTireStepSize(tire_step_size)                              # tire integration step
gator.Initialize()                                                 # build the vehicle

gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)    # simplified to primitives
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # simplified to primitives
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # simplified to primitives
gator.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)      # simplified to primitives
gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)       # simplified to primitives

system = gator.GetSystem()                                          # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED for contact

# simple primitive box collision on the chassis (not a mesh collision)
chassis_body = gator.GetChassisBody()                              # chassis rigid body
chassis_mat = chrono.ChContactMaterialNSC()                        # chassis contact material
chassis_mat.SetFriction(0.9)                                       # friction
chassis_mat.SetRestitution(0.01)                                   # restitution
chassis_body.AddCollisionShape(
    chrono.ChCollisionShapeBox(chassis_mat, 2.0, 1.2, 0.6),        # primitive box, not mesh
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0.3), chrono.QUNIT))   # offset on the chassis
chassis_body.EnableCollision(True)                                 # enable chassis collision
system.GetCollisionSystem().BindAll()                              # bind the new shape

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())               # truth diagnostic banner

terrain = veh.RigidTerrain(system)                                  # flat rigid terrain
patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material
patch_mat.SetFriction(0.9)                                          # terrain friction
patch_mat.SetRestitution(0.01)                                      # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # terrain color
terrain.Initialize()                                               # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle Irrlicht window
vis.SetWindowTitle('Gator vehicle')                                # window title
vis.SetWindowSize(1280, 1024)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)       # chase camera
vis.Initialize()                                                   # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo overlay
vis.AddSkyBox()                                                    # sky box
vis.AddLightDirectional()                                          # vehicle directional light
vis.AttachVehicle(gator.GetVehicle())                             # bind vehicle assets

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive keyboard driver
steering_time = 2.0                                               # slower: 2 s to full steering
throttle_time = 2.0                                               # slower: 2 s to full throttle
braking_time = 1.0                                                # slower: 1 s to full braking
driver.SetSteeringDelta(render_step_size / steering_time)         # less responsive steering
driver.SetThrottleDelta(render_step_size / throttle_time)         # less responsive throttle
driver.SetBrakingDelta(render_step_size / braking_time)           # less responsive braking
driver.Initialize()                                               # build the driver

render_steps = math.ceil(render_step_size / step_size)            # steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
step_number = 0                                                   # physics step counter


while vis.Run():
    time = system.GetChTime()                                    # current sim time

    if step_number % render_steps == 0:                          # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                           # current driver inputs

    driver.Synchronize(time)                                     # update driver
    terrain.Synchronize(time)                                    # update terrain
    gator.Synchronize(time, driver_inputs, terrain)              # update vehicle
    vis.Synchronize(time, driver_inputs)                         # update visuals

    driver.Advance(step_size)                                    # advance driver
    terrain.Advance(step_size)                                   # advance terrain
    gator.Advance(step_size)                                     # advance vehicle (steps system)
    vis.Advance(step_size)                                       # advance visuals

    step_number += 1                                             # next step
    realtime_timer.Spin(step_size)                               # match wall clock
