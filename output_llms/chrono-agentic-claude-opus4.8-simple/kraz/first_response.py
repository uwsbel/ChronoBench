import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # core Chrono data
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data subtree

initLoc = chrono.ChVector3d(0, 0, 0.5)                               # initial vehicle location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                          # QUNIT, no initial rotation

terrainLength = 100.0                                                # terrain X size
terrainWidth = 100.0                                                 # terrain Y size

step_size = 1e-3                                                      # integration step
tire_step_size = step_size                                           # tire integration step
render_step_size = 1.0 / 50.0                                        # 50 FPS render cadence

vehicle = veh.Kraz()                                                 # Kraz tractor-trailer wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                 # rigid terrain -> NSC
vehicle.SetChassisFixed(False)                                       # chassis must be free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))        # spawn pose
vehicle.Initialize()                                                 # build the vehicle subsystems

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # contact scene needs BULLET

print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())             # Kraz mass via tractor

vis_type = veh.VisualizationType_MESH                                # mesh visualization
vehicle.SetChassisVisualizationType(vis_type, vis_type)             # tractor + trailer chassis
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)          # suspensions
vehicle.SetSteeringVisualizationType(vis_type)                      # steering
vehicle.SetWheelVisualizationType(vis_type, vis_type)              # wheels
vehicle.SetTireVisualizationType(vis_type, vis_type)               # tires

terrain = veh.RigidTerrain(vehicle.GetSystem())                     # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material
patch_mat.SetFriction(0.9)                                          # terrain friction
patch_mat.SetRestitution(0.01)                                     # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)          # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # patch color
terrain.Initialize()                                               # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-specific Irrlicht vis
vis.SetWindowTitle('Kraz Demo')                                    # window title
vis.SetWindowSize(1280, 1024)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.1), 25.0, 1.5)        # trackPoint, chase dist, height
vis.Initialize()                                                   # Irrlicht init FIRST
vis.AddLogo()                                                      # logo
vis.AddLightDirectional()                                          # directional light
vis.AddSkyBox()                                                    # sky box
vis.AttachVehicle(vehicle.GetTractor())                            # attach tractor (Kraz)

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive keyboard driver
driver.SetSteeringDelta(render_step_size / 1.0)                    # steering response
driver.SetThrottleDelta(render_step_size / 1.0)                    # throttle response
driver.SetBrakingDelta(render_step_size / 0.3)                     # braking response
driver.Initialize()                                               # init driver

render_steps = math.ceil(render_step_size / step_size)            # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
step_number = 0                                                   # step counter


while vis.Run():
    time = vehicle.GetSystem().GetChTime()                        # current sim time

    if step_number % render_steps == 0:                           # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                            # current driver command

    driver.Synchronize(time)                                      # sync driver
    terrain.Synchronize(time)                                     # sync terrain
    vehicle.Synchronize(time, driver_inputs, terrain)            # sync vehicle
    vis.Synchronize(time, driver_inputs)                          # sync vis

    driver.Advance(step_size)                                     # advance driver
    terrain.Advance(step_size)                                    # advance terrain
    vehicle.Advance(step_size)                                    # advance vehicle (steps the system)
    vis.Advance(step_size)                                        # advance vis

    step_number += 1                                              # increment step
    realtime_timer.Spin(step_size)                                # pace to real time
