import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle data files

initLoc = chrono.ChVector3d(0, 0, 0.5)                             # initial vehicle position
initRot = chrono.QUNIT                                              # initial vehicle orientation

step_size = 1e-3                                                    # physics time step
sim_end = 10.0                                                      # simulation end time (s)
render_fps = 50                                                     # render frames per second
render_step_size = 1.0 / render_fps                                 # render interval (s)
render_steps = math.ceil(render_step_size / step_size)              # steps per render frame

terrainLength = 200.0                                               # terrain patch length (m)
terrainWidth = 200.0                                                # terrain patch width (m)

vehicle = veh.UAZBUS()                                              # UAZBUS catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)             # chassis collision off
vehicle.SetChassisFixed(False)                                      # MANDATORY - chassis must be free
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))      # set initial pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tire model
vehicle.SetTireStepSize(step_size)                                  # tire integration step
vehicle.Initialize()                                                # initialize vehicle

system = vehicle.GetSystem()                                        # get the vehicle-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED - Bullet collision

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)          # mesh chassis
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # primitives suspension
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # primitives steering
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)            # mesh wheels
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)             # mesh tires

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # rigid terrain on vehicle system

patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material
patch_mat.SetFriction(0.9)                                          # friction coefficient
patch_mat.SetRestitution(0.01)                                      # restitution

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,                                                # centered at origin
    terrainLength,
    terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # terrain texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # terrain color
terrain.Initialize()                                                # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle Irrlicht vis system
vis.SetWindowTitle("UAZBUS Vehicle Simulation")                     # window title
vis.SetWindowSize(1280, 1024)                                       # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)   # chase camera settings
vis.Initialize()                                                    # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # Chrono logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                           # directional light
vis.AttachVehicle(vehicle.GetVehicle())                             # attach vehicle to vis

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive keyboard driver

steering_time = 1.0                                                 # time to full steering
throttle_time = 1.0                                                 # time to full throttle
braking_time = 0.3                                                  # time to full braking

driver.SetSteeringDelta(render_step_size / steering_time)           # steering delta per render step
driver.SetThrottleDelta(render_step_size / throttle_time)           # throttle delta per render step
driver.SetBrakingDelta(render_step_size / braking_time)             # braking delta per render step
driver.Initialize()                                                 # initialize driver

render_every = max(1, round(1.0 / (render_fps * step_size)))        # untagged render cadence
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()                       # real-time sync timer

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                       # current simulation time

    if step_number % render_steps == 0:                             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver.Synchronize(time)                                        # synchronize driver
    driver.SetThrottle(0.5)                                         # scripted constant throttle
    driver_inputs = driver.GetInputs()                              # get inputs after scripted throttle

    terrain.Synchronize(time)                                       # synchronize terrain
    vehicle.Synchronize(time, driver_inputs, terrain)               # synchronize vehicle
    vis.Synchronize(time, driver_inputs)                            # synchronize visualization


    driver.Advance(step_size)                                       # advance driver
    terrain.Advance(step_size)                                      # advance terrain
    vehicle.Advance(step_size)                                      # advance vehicle + system
    vis.Advance(step_size)                                          # advance visualization

    step_number += 1
    realtime_timer.Spin(step_size)                                  # real-time synchronization
