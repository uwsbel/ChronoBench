import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # core Chrono data
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data

initLoc = chrono.ChVector3d(-40, 0, 0.5)                             # spawn at -40 along X
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                          # QUNIT (no rotation)

step_size = 1e-3                                                     # integration step
tire_step_size = step_size                                          # tire substep
render_step_size = 1.0 / 50.0                                        # 50 FPS visualization

terrainLength = 100.0                                               # terrain X size
terrainWidth = 100.0                                                # terrain Y size

vehicle = veh.UAZBUS()                                              # UAZBUS catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                # rigid terrain -> NSC
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)            # no chassis collision
vehicle.SetChassisFixed(False)                                      # chassis is free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))       # initial pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                             # tire integration step
vehicle.Initialize()                                                # build subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheel mesh
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)        # tire mesh

system = vehicle.GetSystem()                                        # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # Bullet collision

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # diagnostic

terrain = veh.RigidTerrain(system)                                  # rigid ground
patch_mat = chrono.ChContactMaterialNSC()                           # NSC terrain material
patch_mat.SetFriction(0.9)                                          # friction coefficient
patch_mat.SetRestitution(0.01)                                      # restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)       # concrete texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # patch color
terrain.Initialize()                                                # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle Irrlicht vis
vis.SetWindowTitle('UAZBUS Demo')                                   # window title
vis.SetWindowSize(1280, 1024)                                       # window size
vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)         # chase camera
vis.Initialize()                                                    # build device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # directional light
vis.AttachVehicle(vehicle.GetVehicle())                            # bind vehicle assets

# Double lane change maneuver: timed (time, steering, throttle, braking) entries
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),                        # start at rest
    veh.DataDriverEntry(0.5, 0.0, 0.7, 0.0),                        # accelerate straight
    veh.DataDriverEntry(2.0, 0.0, 0.7, 0.0),                        # steady throttle
    veh.DataDriverEntry(3.0, 0.4, 0.5, 0.0),                        # steer left (lane 1)
    veh.DataDriverEntry(4.0, -0.4, 0.5, 0.0),                       # steer right (back)
    veh.DataDriverEntry(5.0, 0.0, 0.5, 0.0),                        # straighten
    veh.DataDriverEntry(6.0, -0.4, 0.5, 0.0),                       # steer right (lane 2)
    veh.DataDriverEntry(7.0, 0.4, 0.5, 0.0),                        # steer left (back)
    veh.DataDriverEntry(8.0, 0.0, 0.5, 0.0),                        # straighten
    veh.DataDriverEntry(9.0, 0.0, 0.0, 1.0),                        # brake to stop
])
driver = veh.ChDataDriver(vehicle.GetVehicle(), driver_data)        # scripted data driver
driver.Initialize()                                                # init driver

render_steps = math.ceil(render_step_size / step_size)             # steps per frame

realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacing
step_number = 0                                                    # physics step counter
while vis.Run():                                                   # real-time render loop
    time = system.GetChTime()                                      # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver inputs

    driver.Synchronize(time)                                       # sync driver
    terrain.Synchronize(time)                                      # sync terrain
    vehicle.Synchronize(time, driver_inputs, terrain)              # sync vehicle
    vis.Synchronize(time, driver_inputs)                           # sync visuals

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    vehicle.Advance(step_size)                                     # advance vehicle (steps system)
    vis.Advance(step_size)                                         # advance visuals

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # pace to real time
