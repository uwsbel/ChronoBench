import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.4)                               # UAZBUS chassis spawn (origin at geometric center)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation, facing +X
step_size = 1e-3                                                       # integration step
tire_step_size = 1e-3                                                  # tire force-model substep

vehicle = veh.UAZBUS()                                                 # UAZ bus catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision mesh
vehicle.SetChassisFixed(False)                                        # MANDATORY — fixed chassis never moves
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial pose
vehicle.SetTireType(veh.TireModelType_RIGID)                         # prompt: rigid tire model
vehicle.SetTireStepSize(tire_step_size)                              # tire substep
vehicle.Initialize()                                                  # build the vehicle subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)   # chassis mesh
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension links
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering links
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)     # wheel rims
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)      # tire meshes

system = vehicle.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact/terrain
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                    # flat rigid road under the bus
patch_mat = chrono.ChContactMaterialNSC()                            # NSC patch material
patch_mat.SetFriction(0.9)                                            # tire-road friction
patch_mat.SetRestitution(0.01)                                        # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)   # 100x100 m flat patch at origin
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                        # tan road color
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
terrain.Initialize()                                                  # build the terrain

box_mat = chrono.ChContactMaterialNSC()                              # obstacle contact material
box_mat.SetFriction(0.9)                                              # obstacle friction
box = chrono.ChBodyEasyBox(0.5, 5, 0.2, 1000, True, True, box_mat)   # box obstacle 0.5 x 5 x 0.2
box.SetPos(chrono.ChVector3d(5, 0, 0.1))                             # placed ahead of the bus
box.SetFixed(True)                                                    # fixed obstacle to test mobility
system.AddBody(box)                                                   # add obstacle to the world

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht system
vis.SetWindowTitle("UAZBUS over box obstacle")                       # window title
vis.SetWindowSize(1280, 1024)                                         # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)         # chase camera on the chassis
vis.Initialize()                                                      # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo
vis.AddSkyBox()                                                       # sky backdrop
vis.AddLightDirectional()                                            # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())                             # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive driver bound to the vis
steering_time = 1.0                                                   # s to reach full steering
throttle_time = 1.0                                                   # s to reach full throttle
braking_time = 0.3                                                    # s to reach full braking
driver.SetSteeringDelta(step_size / steering_time)                  # steering ramp rate
driver.SetThrottleDelta(step_size / throttle_time)                  # throttle ramp rate
driver.SetBrakingDelta(step_size / braking_time)                    # braking ramp rate
driver.Initialize()                                                  # build the driver

render_step_size = 1.0 / 50.0                                        # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)              # physics steps per rendered frame
render_every = render_steps                                          # untagged cadence constant

sim_end = 12.0                                                       # total simulated duration
realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
step_number = 0                                                      # physics step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                       # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver.SetThrottle(0.5)                                         # prompt: constant forward throttle 0.5
    driver_inputs = driver.GetInputs()                             # current driver command

    driver.Synchronize(time)                                        # driver state
    terrain.Synchronize(time)                                       # terrain state
    vehicle.Synchronize(time, driver_inputs, terrain)             # vehicle reads driver + terrain
    vis.Synchronize(time, driver_inputs)                          # vis HUD

    driver.Advance(step_size)                                       # advance driver
    terrain.Advance(step_size)                                      # advance terrain
    vehicle.Advance(step_size)                                      # advances the wrapper-owned system
    vis.Advance(step_size)                                          # advance vis


    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # spin so wall-clock matches sim time
